/*=Plus=header=begin======================================================
  Program: Plus
  Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
  See License.txt for details.
=========================================================Plus=header=end*/

#include "PlusConfigure.h"
#include <limits.h>
#include <float.h>
#include <math.h>
#include <sstream>
#include "vtkCharArray.h"
#include "vtkRecursiveCriticalSection.h"
#include "vtkDoubleArray.h"
#include "vtkMatrix4x4.h"
#include "vtkMultiThreader.h"
#include "vtkObjectFactory.h"
#include "vtkSocketCommunicator.h" // VTK_PARALLEL support has to be enabled
#include "vtkTracker.h"
#include "vtkTransform.h"
#include "vtkTimerLog.h"
#include "vtkTrackerTool.h"
#include "vtkTrackerBuffer.h"
#include "vtksys/SystemTools.hxx"
#include "vtkAccurateTimer.h"
#include "vtkGnuplotExecuter.h"
#include "vtkHTMLGenerator.h"
#include "vtkTrackedFrameList.h"
#include "TrackedFrame.h"

vtkStandardNewMacro(vtkTracker);

//----------------------------------------------------------------------------
vtkTracker::vtkTracker()
{
  this->InternalUpdateRate = 0;
  this->AcquisitionRate = 50; 
  this->ToolReferenceFrameName = NULL; 
  this->TrackingThreadAlive = false; 
  
  // For threaded capture of transformations
  this->UpdateMutex = vtkRecursiveCriticalSection::New();

  this->SetToolReferenceFrameName("Tracker"); 
}

//----------------------------------------------------------------------------
vtkTracker::~vtkTracker()
{
  this->StopTracking();
  this->Disconnect();

  for ( ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it )
  {
    it->second->SetTracker(NULL); 
    it->second->Delete(); 
  }

  DELETE_IF_NOT_NULL(this->UpdateMutex); 
}

//----------------------------------------------------------------------------
void vtkTracker::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  if (this->ToolReferenceFrameName)
  {
    os << indent << "ToolReferenceFrameName: " << this->ToolReferenceFrameName << "\n";
  }
}

//----------------------------------------------------------------------------
ToolIteratorType vtkTracker::GetToolIteratorBegin()
{
  return this->ToolContainer.begin(); 
}

//----------------------------------------------------------------------------
ToolIteratorType vtkTracker::GetToolIteratorEnd()
{
  return this->ToolContainer.end();
}

//----------------------------------------------------------------------------
int vtkTracker::GetNumberOfTools()
{
  return this->ToolContainer.size(); 
}


//----------------------------------------------------------------------------
PlusStatus vtkTracker::AddTool( vtkTrackerTool* tool )
{
  if ( tool == NULL )
  {
    LOG_ERROR("Failed to add tool to tracker, tool is NULL!"); 
    return PLUS_FAIL; 
  }

  if ( tool->GetToolName() == NULL || tool->GetPortName() == NULL )
  {
    LOG_ERROR("Failed to add tool to tracker, tool Name and PortName must be defined!"); 
    return PLUS_FAIL; 
  }

  if ( this->ToolContainer.find( tool->GetToolName() ) == this->GetToolIteratorEnd() )
  {
    // Check tool port names, it should be unique too
    for ( ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it)
    {
      if ( STRCASECMP( tool->GetPortName(), it->second->GetPortName() ) == 0 )
      {
        LOG_ERROR("Failed to add '" << tool->GetToolName() << "' tool to container: tool with name '" << it->second->GetToolName() 
          << "' is already defined on port '" << tool->GetPortName() << "'!"); 
        return PLUS_FAIL; 
      }
    }

    tool->Register(this); 
    tool->SetTracker(this); 
    this->ToolContainer[tool->GetToolName()] = tool; 
  }
  else
  {
    LOG_ERROR("Tool with name '" << tool->GetToolName() << "' is already in the tool conatainer!"); 
    return PLUS_FAIL; 
  }

  return PLUS_SUCCESS; 
}

//----------------------------------------------------------------------------
PlusStatus vtkTracker::GetFirstActiveTool(vtkTrackerTool* &aTool)
{
  if ( this->GetToolIteratorBegin() == this->GetToolIteratorEnd() )
  {
    LOG_ERROR("Failed to get first active tool - there is no active tool!"); 
    return PLUS_FAIL; 
  }

  // Get the first tool
  aTool = this->GetToolIteratorBegin()->second; 

  return PLUS_SUCCESS; 
}

//----------------------------------------------------------------------------
PlusStatus vtkTracker::GetTool(const char* aToolName, vtkTrackerTool* &aTool)
{
  if ( aToolName == NULL )
  {
    LOG_ERROR("Failed to get tool, tool name is NULL!"); 
    return PLUS_FAIL; 
  }

  ToolIteratorType tool = this->ToolContainer.find(aToolName); 
  if ( tool == this->GetToolIteratorEnd() )
  {
    std::ostringstream availableTools; 
    for ( ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it )
    {
      availableTools << it->first <<";"; 
    }
    LOG_ERROR("Unable to find tool '"<< aToolName <<"' in the list, please check the configuration file first (available tools: " << availableTools.str() << ")." ); 
    return PLUS_FAIL; 
  }

  aTool = tool->second; 

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
// this thread is run whenever the tracker is tracking
void * vtkTracker::vtkTrackerThread(vtkMultiThreader::ThreadInfo *data)
{
  vtkTracker *self = (vtkTracker *)(data->UserData);

  self->TrackingThreadAlive = true; 

  double currtime[10]={0};
  unsigned long updatecount = 0; 
  // loop until cancelled
  while ( self->Recording )
  {
    double newtime = vtkAccurateTimer::GetSystemTime(); 

    // get current tracking rate over last 10 updates
    double difftime = newtime - currtime[updatecount%10];
    currtime[updatecount%10] = newtime;
    if (updatecount > 10 && difftime != 0)
    {
      self->InternalUpdateRate = (10.0/difftime);
    }    

    { // Lock before update 
      PlusLockGuard<vtkRecursiveCriticalSection> updateMutexGuardedLock(self->UpdateMutex);
      if (!self->Recording)
      {
        // recording has been stopped
        break;
      }
      self->InternalUpdate();
      self->UpdateTime.Modified();
    }

    double delay = ( newtime + 1.0 / self->GetAcquisitionRate() - vtkAccurateTimer::GetSystemTime() );
    if ( delay > 0 )
    {
      vtkAccurateTimer::Delay(delay); 
    }

    updatecount++;
  }

  LOG_DEBUG("Stopped tracking");
  self->TrackingThreadAlive = false; 
  return NULL;

}

//----------------------------------------------------------------------------
PlusStatus vtkTracker::Probe()
{
  PlusLockGuard<vtkRecursiveCriticalSection> updateMutexGuardedLock(this->UpdateMutex);

  // Client

  if (this->InternalStartTracking() != PLUS_SUCCESS)
  {
    return PLUS_FAIL;
  }

  if (this->InternalStopTracking() != PLUS_SUCCESS)
  {
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkTracker::StartTracking()
{
  // start the tracking thread
  if ( this->Recording )
  {
    LOG_ERROR("Cannot start the tracking thread - tracking still running!");
    return PLUS_FAIL;
  }

  if ( this->InternalStartTracking() != PLUS_SUCCESS )
  {
    LOG_ERROR("Failed to start tracking!"); 
    return PLUS_FAIL; 
  } 

  this->Recording = 1;

  // start the tracking thread
  this->ThreadId = this->Threader->SpawnThread((vtkThreadFunctionType)\
    &vtkTrackerThread,this);
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkTracker::StopTracking()
{

  if (!this->Recording)
  {
    // tracking already stopped, nothing to do
    return PLUS_SUCCESS;
  }

  this->ThreadId = -1;
  this->Recording = 0;

  // Let's give a chance to the thread to stop before we kill the tracker connection
  while ( this->TrackingThreadAlive )
  {
    vtkAccurateTimer::Delay(0.1);
  }

  if ( this->InternalStopTracking() != PLUS_SUCCESS )
  {
    LOG_ERROR("Failed to stop tracking thread!"); 
    return PLUS_FAIL; 
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkTracker::ToolTimeStampedUpdateWithoutFiltering(const char* aToolName, vtkMatrix4x4 *matrix, ToolStatus status, double unfilteredtimestamp, double filteredtimestamp) 
{
  if ( aToolName == NULL )
  {
    LOG_ERROR("Failed to update tool - tool name is NULL!"); 
    return PLUS_FAIL; 
  }

  vtkTrackerTool* tool = NULL; 
  if ( this->GetTool(aToolName, tool) != PLUS_SUCCESS )
  {
    LOG_ERROR("Failed to update tool - unable to find tool!" << aToolName ); 
    return PLUS_FAIL; 
  }
  
  // This function is for devices has no frame numbering, just auto increment tool frame number if new frame received
  unsigned long frameNumber = tool->GetFrameNumber() + 1 ; 
  vtkTrackerBuffer *buffer = tool->GetBuffer();
  PlusStatus bufferStatus = buffer->AddTimeStampedItem(matrix, status, frameNumber, unfilteredtimestamp, filteredtimestamp);
  tool->SetFrameNumber(frameNumber); 

  return bufferStatus; 
}

//----------------------------------------------------------------------------
PlusStatus vtkTracker::ToolTimeStampedUpdate(const char* aToolName, vtkMatrix4x4 *matrix, ToolStatus status, unsigned long frameNumber, double unfilteredtimestamp) 
{
  if ( aToolName == NULL )
  {
    LOG_ERROR("Failed to update tool - tool name is NULL!"); 
    return PLUS_FAIL; 
  }

  vtkTrackerTool* tool = NULL; 
  if ( this->GetTool(aToolName, tool) != PLUS_SUCCESS )
  {
    LOG_ERROR("Failed to update tool - unable to find tool!" << aToolName ); 
    return PLUS_FAIL; 
  }

  vtkTrackerBuffer *buffer = tool->GetBuffer();
  PlusStatus bufferStatus = buffer->AddTimeStampedItem(matrix, status, frameNumber, unfilteredtimestamp);
  tool->SetFrameNumber(frameNumber); 

  return bufferStatus; 
}

//----------------------------------------------------------------------------
void vtkTracker::Beep(int n)
{

  PlusLockGuard<vtkRecursiveCriticalSection> updateMutexGuardedLock(this->UpdateMutex);
  this->InternalBeep(n);
}

//----------------------------------------------------------------------------
void vtkTracker::SetToolLED(const char* portName, int led, int state)
{
  PlusLockGuard<vtkRecursiveCriticalSection> updateMutexGuardedLock(this->UpdateMutex);
  this->InternalSetToolLED(portName, led, state);
}

//-----------------------------------------------------------------------------
PlusStatus vtkTracker::Connect()
{
  return PLUS_SUCCESS; 
}

//-----------------------------------------------------------------------------
PlusStatus vtkTracker::Disconnect()
{
  return PLUS_SUCCESS;
}

//-----------------------------------------------------------------------------
void vtkTracker::DeepCopy(vtkTracker *tracker)
{
  LOG_TRACE("vtkTracker::DeepCopy"); 
  for ( ToolIteratorType it = tracker->ToolContainer.begin(); it != tracker->ToolContainer.end(); ++it )
  {
    LOG_DEBUG("Copy the buffer of tracker tool: " << it->first ); 
    if ( this->AddTool(it->second) != PLUS_SUCCESS )
    {
      LOG_ERROR("Copy of tool '" << it->first << "' failed - unabale to add tool to the container!"); 
      continue; 
    }
    
    vtkTrackerTool* tool = NULL; 
    if ( this->GetTool(it->first.c_str(), tool ) != PLUS_SUCCESS )
    {
      LOG_ERROR("Copy of tool '" << it->first << "' failed - unabale to get tool from container!"); 
      continue;   
    }

    tool->DeepCopy( it->second ); 
  }

  this->InternalUpdateRate = tracker->GetInternalUpdateRate();
  this->SetAcquisitionRate(tracker->GetAcquisitionRate()); 
}

//-----------------------------------------------------------------------------
PlusStatus vtkTracker::WriteConfiguration(vtkXMLDataElement* config)
{
  if ( config == NULL )
  {
    LOG_ERROR("Unable to write configuration: xml data element is NULL!"); 
    return PLUS_FAIL;
  }

	vtkXMLDataElement* dataCollectionConfig = config->FindNestedElementWithName("DataCollection");
	if (dataCollectionConfig == NULL)
  {
    LOG_ERROR("Cannot find DataCollection element in XML tree!");
		return PLUS_FAIL;
	}

  vtkXMLDataElement* trackerConfig = dataCollectionConfig->FindNestedElementWithName("Tracker"); 
  if (trackerConfig == NULL) 
  {
    LOG_ERROR("Cannot find Tracker element in XML tree!");
		return PLUS_FAIL;
  }

  if ( trackerConfig == NULL )
  {
    LOG_ERROR("Unable to find Tracker xml data element!"); 
    return PLUS_FAIL; 
  }

  if ( !this->ToolContainer.empty() )  
  {
    vtkTrackerTool* tool = this->GetToolIteratorBegin()->second; 

    trackerConfig->SetIntAttribute("BufferSize", tool->GetBuffer()->GetBufferSize()); 

    trackerConfig->SetDoubleAttribute("LocalTimeOffsetSec", tool->GetBuffer()->GetLocalTimeOffsetSec() ); 
  }

  return PLUS_SUCCESS; 
}

//-----------------------------------------------------------------------------
void vtkTracker::SetToolsBufferSize( int aBufferSize )
{
  LOG_TRACE("vtkTracker::SetToolsBufferSize to " << aBufferSize ); 
  for ( ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it)
  {
    it->second->GetBuffer()->SetBufferSize( aBufferSize ); 
  }
}

//-----------------------------------------------------------------------------
void vtkTracker::SetToolsLocalTimeOffsetSec( double aLocalTimeOffsetSec )
{
  LOG_INFO("Tools local time offset: " << 1000*aLocalTimeOffsetSec << "ms" ); 
  for ( ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it)
  {
    it->second->GetBuffer()->SetLocalTimeOffsetSec(aLocalTimeOffsetSec); 
  }
}


//-----------------------------------------------------------------------------
PlusStatus vtkTracker::ReadConfiguration(vtkXMLDataElement* config)
{
  LOG_TRACE("vtkTracker::ReadConfiguration"); 
  if ( config == NULL )
  {
    LOG_ERROR("Unable to configure tracker! (XML data element is NULL)"); 
    return PLUS_FAIL; 
  }

  vtkXMLDataElement* dataCollectionConfig = config->FindNestedElementWithName("DataCollection");
	if (dataCollectionConfig == NULL)
  {
    LOG_ERROR("Cannot find DataCollection element in XML tree!");
		return PLUS_FAIL;
	}

  vtkXMLDataElement* trackerConfig = dataCollectionConfig->FindNestedElementWithName("Tracker"); 
  if (trackerConfig == NULL) 
  {
    LOG_ERROR("Cannot find Tracker element in XML tree!");
		return PLUS_FAIL;
  }

  // Read tool configurations 
  for ( int tool = 0; tool < trackerConfig->GetNumberOfNestedElements(); tool++ )
  {
    vtkXMLDataElement* toolDataElement = trackerConfig->GetNestedElement(tool); 
    if ( STRCASECMP(toolDataElement->GetName(), "Tool") != 0 )
    {
      // if this is not a Tool element, skip it
      continue; 
    }

    vtkSmartPointer<vtkTrackerTool> trackerTool = vtkSmartPointer<vtkTrackerTool>::New(); 
    if ( trackerTool->ReadConfiguration(toolDataElement) != PLUS_SUCCESS )
    {
      LOG_ERROR("Unable to add tool to tracker - failed to read tool configuration"); 
      continue; 
    }

    if ( this->AddTool(trackerTool) != PLUS_SUCCESS )
    {
      LOG_ERROR("Failed to add tool '" << trackerTool->GetToolName() << "' to tracker on port " << trackerTool->GetPortName() );
      continue; 
    }
  }

  int bufferSize = 0; 
  if ( trackerConfig->GetScalarAttribute("BufferSize", bufferSize) ) 
  {
    this->SetToolsBufferSize(bufferSize); 
  }

  double acquisitionRate = 0; 
  if ( trackerConfig->GetScalarAttribute("AcquisitionRate", acquisitionRate) ) 
  {
    this->SetAcquisitionRate(acquisitionRate);  
  }

  int averagedItemsForFiltering = 0; 
  if ( trackerConfig->GetScalarAttribute("AveragedItemsForFiltering", averagedItemsForFiltering) )
  {
    for ( ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it)
    {
      it->second->GetBuffer()->SetAveragedItemsForFiltering(averagedItemsForFiltering); 
    }
  }
  else
  {
    LOG_WARNING("Unable to find Tracker AveragedItemsForFiltering attribute in configuration file!"); 
  }

  double localTimeOffsetSec = 0; 
  if ( trackerConfig->GetScalarAttribute("LocalTimeOffsetSec", localTimeOffsetSec) )
  {
    this->SetToolsLocalTimeOffsetSec(localTimeOffsetSec); 
  }
  
  return PLUS_SUCCESS; 
}

//-----------------------------------------------------------------------------
PlusStatus vtkTracker::GetToolByPortName( const char* portName, vtkTrackerTool* &aTool)
{
  if ( portName == NULL )
  {
    LOG_ERROR("Failed to get tool - port name is NULL!"); 
    return PLUS_FAIL; 
  }

  for ( ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it)
  {
    if ( STRCASECMP( portName, it->second->GetPortName() ) == 0 )
    {
      aTool = it->second; 
      return PLUS_SUCCESS; 
    }
  }

  return PLUS_FAIL; 
}

//----------------------------------------------------------------------------
void vtkTracker::SetStartTime( double startTime)
{
  for ( ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it)
  {
    it->second->GetBuffer()->SetStartTime(startTime); 
  }
}

//----------------------------------------------------------------------------
double vtkTracker::GetStartTime()
{
  double sumStartTime = 0.0;
  double numberOfTools(0); 
  for ( ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it)
  {
    sumStartTime += it->second->GetBuffer()->GetStartTime(); 
    numberOfTools++; 
  }

  return sumStartTime / numberOfTools;
}

//----------------------------------------------------------------------------
TrackedFrameFieldStatus vtkTracker::ConvertToolStatusToTrackedFrameFieldStatus(ToolStatus status)
{
  TrackedFrameFieldStatus fieldStatus = FIELD_INVALID; 
  if ( status == TOOL_OK )
  {
    fieldStatus = FIELD_OK; 
  }

  return fieldStatus; 
}

//----------------------------------------------------------------------------
ToolStatus vtkTracker::ConvertTrackedFrameFieldStatusToToolStatus(TrackedFrameFieldStatus fieldStatus)
{
  ToolStatus status = TOOL_MISSING; 
  if ( fieldStatus == FIELD_OK)
  {
    status = TOOL_OK ; 
  }

  return status; 
}

//----------------------------------------------------------------------------
std::string vtkTracker::ConvertToolStatusToString(ToolStatus status)
{
  std::string flagFieldValue; 
  if ( status == TOOL_OK )
  {
    flagFieldValue = "OK"; 
  }
  else if ( status == TOOL_MISSING )
  {
    flagFieldValue = "TOOL_MISSING"; 
  }
  else if ( status == TOOL_OUT_OF_VIEW )
  {
    flagFieldValue = "TOOL_OUT_OF_VIEW"; 
  }
  else if ( status == TOOL_OUT_OF_VOLUME )
  {
    flagFieldValue = "TOOL_OUT_OF_VOLUME"; 
  }
  else if ( status == TOOL_REQ_TIMEOUT )
  {
    flagFieldValue = "TOOL_REQ_TIMEOUT"; 
  }
  else
  { 
    LOG_ERROR("Unknown tracker status received - set TOOL_INVALID by default!"); 
    flagFieldValue = "TOOL_INVALID"; 
  }

  return flagFieldValue; 
}

//----------------------------------------------------------------------------
PlusStatus vtkTracker::GetTrackedFrame(double timestamp, TrackedFrame *aTrackedFrame)
{
  int numberOfErrors(0); 

  if (!aTrackedFrame)
  {
    return PLUS_FAIL;
  }

  // Add main tool timestamp
  std::ostringstream timestampFieldValue; 
  timestampFieldValue << std::fixed << timestamp; 
  aTrackedFrame->SetCustomFrameField("Timestamp", timestampFieldValue.str()); 

  for (ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it)
  {
    PlusTransformName toolTransformName(it->second->GetToolName(), this->ToolReferenceFrameName ); 
    if ( ! toolTransformName.IsValid() )
    {
      LOG_ERROR("Tool transform name is invalid!"); 
      numberOfErrors++; 
      continue; 
    }
    
    TrackerBufferItem bufferItem; 
    if ( it->second->GetBuffer()->GetTrackerBufferItemFromTime(timestamp, &bufferItem, vtkTrackerBuffer::INTERPOLATED ) != ITEM_OK )
    {
      double latestTimestamp(0); 
      if ( it->second->GetBuffer()->GetLatestTimeStamp(latestTimestamp) != ITEM_OK )
      {
        LOG_ERROR("Failed to get latest timestamp!"); 
      }

      double oldestTimestamp(0); 
      if ( it->second->GetBuffer()->GetOldestTimeStamp(oldestTimestamp) != ITEM_OK )
      {
        LOG_ERROR("Failed to get oldest timestamp!"); 
      }

      LOG_ERROR("Failed to get tracker item from buffer by time: " << std::fixed << timestamp << " (Latest timestamp: " << latestTimestamp << "   Oldest timestamp: " << oldestTimestamp << ")."); 
      numberOfErrors++; 
      continue; 
    }

    vtkSmartPointer<vtkMatrix4x4> dMatrix=vtkSmartPointer<vtkMatrix4x4>::New();
    if (bufferItem.GetMatrix(dMatrix)!=PLUS_SUCCESS)
    {
      LOG_ERROR("Failed to get matrix from buffer item for tool " << it->second->GetToolName() ); 
      numberOfErrors++; 
      continue; 
    }

    if ( aTrackedFrame->SetCustomFrameTransform(toolTransformName, dMatrix) != PLUS_SUCCESS )
    {
      LOG_ERROR("Failed to set transform for tool " << it->second->GetToolName() ); 
      numberOfErrors++; 
      continue; 
    }

    if ( aTrackedFrame->SetCustomFrameTransformStatus(toolTransformName, vtkTracker::ConvertToolStatusToTrackedFrameFieldStatus(bufferItem.GetStatus()) ) != PLUS_SUCCESS )
    {
      LOG_ERROR("Failed to set transform status for tool " << it->second->GetToolName() ); 
      numberOfErrors++; 
      continue; 
    }
  }

  return (numberOfErrors == 0 ? PLUS_SUCCESS : PLUS_FAIL ); 
}

//-----------------------------------------------------------------------------
PlusStatus vtkTracker::GenerateTrackingDataAcquisitionReport( vtkHTMLGenerator* htmlReport, vtkGnuplotExecuter* plotter)
{
  if ( htmlReport == NULL || plotter == NULL )
  {
    LOG_ERROR("Caller should define HTML report generator and gnuplot plotter before report generation!"); 
    return PLUS_FAIL; 
  }

  if ( this->ToolContainer.empty() )
  {
    LOG_ERROR("Failed to generate tracking data acqusition report - no tools available!"); 
    return PLUS_FAIL;
  }

  // Use the first tool in the container to generate the report
  vtkTrackerTool* tool = this->GetToolIteratorBegin()->second;  

  vtkSmartPointer<vtkTable> timestampReportTable = vtkSmartPointer<vtkTable>::New(); 
 
  if ( tool->GetBuffer()->GetTimeStampReportTable(timestampReportTable) != PLUS_SUCCESS )
  { 
    LOG_ERROR("Failed to get timestamp report table from tool '"<< tool->GetToolName() << "' buffer!"); 
    return PLUS_FAIL; 
  }

  std::string reportFile = vtkPlusConfig::GetInstance()->GetOutputDirectory() + std::string("/")
    + std::string(vtkPlusConfig::GetInstance()->GetApplicationStartTimestamp()) 
    + std::string(".TrackerBufferTimestamps.txt"); 

  if ( vtkGnuplotExecuter::DumpTableToFileInGnuplotFormat( timestampReportTable, reportFile.c_str() ) != PLUS_SUCCESS )
  {
    LOG_ERROR("Failed to write table to file in gnuplot format!"); 
    return PLUS_FAIL; 
  } 

  if ( !vtksys::SystemTools::FileExists( reportFile.c_str(), true) )
  {
    LOG_ERROR("Unable to find tracking data acquisition report file at: " << reportFile); 
    return PLUS_FAIL; 
  }

  const char* scriptsFolder = vtkPlusConfig::GetInstance()->GetScriptsDirectory();
  std::string plotBufferTimestampScript = scriptsFolder + std::string("/gnuplot/PlotBufferTimestamp.gnu"); 
  if ( !vtksys::SystemTools::FileExists( plotBufferTimestampScript.c_str(), true) )
  {
    LOG_ERROR("Unable to find gnuplot script at: " << plotBufferTimestampScript); 
    return PLUS_FAIL; 
  }

  htmlReport->AddText("Tracking Data Acquisition Analysis", vtkHTMLGenerator::H1); 
  plotter->ClearArguments(); 
  plotter->AddArgument("-e");
  std::ostringstream trackerBufferAnalysis; 
  trackerBufferAnalysis << "f='" << reportFile << "'; o='TrackerBufferTimestamps';" << std::ends; 
  plotter->AddArgument(trackerBufferAnalysis.str().c_str()); 
  plotter->AddArgument(plotBufferTimestampScript.c_str());  
  if ( plotter->Execute() != PLUS_SUCCESS )
  {
    LOG_ERROR("Failed to run gnuplot executer!"); 
    return PLUS_FAIL; 
  } 
  htmlReport->AddImage("TrackerBufferTimestamps.jpg", "Tracking Data Acquisition Analysis"); 

  htmlReport->AddHorizontalLine(); 

  return PLUS_SUCCESS; 
}

//-----------------------------------------------------------------------------
void vtkTracker::ClearAllBuffers()
{
  for ( ToolIteratorType it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it)
  {
    it->second->GetBuffer()->Clear(); 
  }
}

//----------------------------------------------------------------------------
PlusStatus vtkTracker::CopyBuffer( vtkTrackerBuffer* aTrackerBuffer, const char* aToolName )
{
  LOG_TRACE("vtkTracker::CopyBuffer"); 

  if ( aTrackerBuffer == NULL )
  {
    LOG_ERROR("Unable to copy tracker buffer to a NULL buffer!"); 
    return PLUS_FAIL; 
  }

  if ( aToolName == NULL )
  {
    LOG_ERROR("Unable to copy tracker buffer - tool name is NULL!"); 
    return PLUS_FAIL; 
  }

  vtkTrackerTool * tool = NULL; 
  if ( GetTool(aToolName, tool) != PLUS_SUCCESS )
  {
    LOG_ERROR("Failed to get tool with name: " << aToolName ); 
    return PLUS_FAIL; 
  }

  aTrackerBuffer->DeepCopy(tool->GetBuffer()); 

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkTracker::WriteToMetafile( const char* outputFolder, const char* metaFileName, bool useCompression /*= false*/ )
{
  LOG_TRACE("vtkTracker::WriteToMetafile: " << outputFolder << "/" << metaFileName); 

  if ( this->GetNumberOfTools() == 0 )
  {
    LOG_ERROR("Failed to write tracker to metafile - there are no active tools!"); 
    return PLUS_FAIL; 
  }

  // Get the number of items from buffers and use the lowest
  int numberOfItems(-1); 
  for ( ToolIteratorType it = this->ToolContainer.begin(); it != this->ToolContainer.end(); ++it)
  {
    if ( numberOfItems < 0 || numberOfItems > it->second->GetBuffer()->GetNumberOfItems() )
    {
      numberOfItems = it->second->GetBuffer()->GetNumberOfItems(); 
    }
  }

  vtkSmartPointer<vtkTrackedFrameList> trackedFrameList = vtkSmartPointer<vtkTrackedFrameList>::New(); 

  PlusStatus status=PLUS_SUCCESS;

  // Get the first tool
  vtkTrackerTool* firstActiveTool = this->ToolContainer.begin()->second; 

  for ( int i = 0 ; i < numberOfItems; i++ ) 
  {
    //Create fake image 
    typedef itk::Image<unsigned char, 2> ImageType;
    ImageType::Pointer frame = ImageType::New(); 
    ImageType::SizeType size={{1,1}};
    ImageType::IndexType start={{0,0}};
    ImageType::RegionType region;
    region.SetSize(size);
    region.SetIndex(start);
    frame->SetRegions(region);

    try
    {
      frame->Allocate();
    }
    catch (itk::ExceptionObject & err) 
    {		
      LOG_ERROR("Unable to allocate memory for image: " << err.GetDescription() );
      status=PLUS_FAIL;
      continue; 
    }	

    TrackedFrame trackedFrame; 
    trackedFrame.GetImageData()->SetITKImageBase(frame);

    TrackerBufferItem bufferItem; 
    BufferItemUidType uid = firstActiveTool->GetBuffer()->GetOldestItemUidInBuffer() + i; 

    if ( firstActiveTool->GetBuffer()->GetTrackerBufferItem(uid, &bufferItem) != ITEM_OK )
    {
      LOG_ERROR("Failed to get tracker buffer item with UID: " << uid ); 
      continue; 
    }

    const double frameTimestamp = bufferItem.GetFilteredTimestamp(firstActiveTool->GetBuffer()->GetLocalTimeOffsetSec()); 

    // Add main tool timestamp
    std::ostringstream timestampFieldValue; 
    timestampFieldValue << std::fixed << frameTimestamp; 
    trackedFrame.SetCustomFrameField("Timestamp", timestampFieldValue.str()); 

    // Add main tool unfiltered timestamp
    std::ostringstream unfilteredtimestampFieldValue; 
    unfilteredtimestampFieldValue << std::fixed << bufferItem.GetUnfilteredTimestamp(firstActiveTool->GetBuffer()->GetLocalTimeOffsetSec()); 
    trackedFrame.SetCustomFrameField("UnfilteredTimestamp", unfilteredtimestampFieldValue.str()); 

    // Add main tool frameNumber
    std::ostringstream frameNumberFieldValue; 
    frameNumberFieldValue << std::fixed << bufferItem.GetIndex(); 
    trackedFrame.SetCustomFrameField("FrameNumber", frameNumberFieldValue.str()); 


    // Add transforms
    for ( ToolIteratorType it = this->ToolContainer.begin(); it != this->ToolContainer.end(); ++it)
    {
      TrackerBufferItem toolBufferItem; 
      if ( it->second->GetBuffer()->GetTrackerBufferItemFromTime( frameTimestamp, &toolBufferItem, vtkTrackerBuffer::EXACT_TIME ) != ITEM_OK )
      {
        LOG_ERROR("Failed to get tracker buffer item from time: " << std::fixed << frameTimestamp ); 
        continue; 
      }

      vtkSmartPointer<vtkMatrix4x4> toolMatrix=vtkSmartPointer<vtkMatrix4x4>::New();
      if (toolBufferItem.GetMatrix(toolMatrix)!=PLUS_SUCCESS)
      {
        LOG_ERROR("Failed to get toolMatrix"); 
        return PLUS_FAIL; 
      }
  
      PlusTransformName toolToTrackerTransform(it->second->GetToolName(), this->ToolReferenceFrameName ); 
      trackedFrame.SetCustomFrameTransform(toolToTrackerTransform, toolMatrix ); 

      // Add tool status
      trackedFrame.SetCustomFrameTransformStatus(toolToTrackerTransform, vtkTracker::ConvertToolStatusToTrackedFrameFieldStatus(toolBufferItem.GetStatus()) ); 
    }

    // Add tracked frame to the list
    trackedFrameList->AddTrackedFrame(&trackedFrame); 
  }

  // Save tracked frames to metafile
  if ( trackedFrameList->SaveToSequenceMetafile(outputFolder, metaFileName, vtkTrackedFrameList::SEQ_METAFILE_MHA, useCompression) != PLUS_SUCCESS )
  {
    LOG_ERROR("Failed to save tracked frames to sequence metafile!"); 
    return PLUS_FAIL;
  }

  return status;
}
