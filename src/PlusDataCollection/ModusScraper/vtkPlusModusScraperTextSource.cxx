/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

// Local includes
#include "PlusConfigure.h"
#include "vtkPlusModusScraperTextSource.h"
#include "vtkPlusChannel.h"
#include "vtkPlusDataSource.h"

// VTK includes
#include <vtkImageData.h>
#include <vtkObjectFactory.h>

//----------------------------------------------------------------------------

vtkStandardNewMacro(vtkPlusModusScraperTextSource);

//----------------------------------------------------------------------------
vtkPlusModusScraperTextSource::vtkPlusModusScraperTextSource()
  : URL("")
{
  this->StartThreadForInternalUpdates = true;
}

//----------------------------------------------------------------------------
vtkPlusModusScraperTextSource::~vtkPlusModusScraperTextSource()
{
}

//----------------------------------------------------------------------------
void vtkPlusModusScraperTextSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "URL: " << this->URL<< std::endl;
}

//-----------------------------------------------------------------------------
PlusStatus vtkPlusModusScraperTextSource::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusModusScraperTextSource::ReadConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  XML_READ_STRING_ATTRIBUTE_OPTIONAL(URL, deviceConfig);

  return PLUS_SUCCESS;
}

//-----------------------------------------------------------------------------
PlusStatus vtkPlusModusScraperTextSource::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusOpenCVCaptureVideoSource::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);

  XML_WRITE_STRING_ATTRIBUTE_REMOVE_IF_EMPTY(URL, deviceConfig);

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusModusScraperTextSource::FreezeDevice(bool freeze)
{
  if (freeze)
  {
    this->Disconnect();
  }
  else
  {
    this->Connect();
  }
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusModusScraperTextSource::InternalConnect()
{
  if (!this->URL.empty())
  {
    
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusModusScraperTextSource::InternalDisconnect()
{

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusModusScraperTextSource::InternalUpdate()
{
  LOG_TRACE("vtkPlusModusScraperTextSource::InternalUpdate");

  /*
  vtkPlusDataSource* aSource(nullptr);
  if (this->GetFirstActiveOutputVideoSource(aSource) == PLUS_FAIL || aSource == nullptr)
  {
    LOG_ERROR("Unable to grab a video source. Skipping frame.");
    return PLUS_FAIL;
  }

  if (aSource->GetNumberOfItems() == 0)
  {
    // Init the buffer with the metadata from the first frame
    aSource->SetImageType(US_IMG_RGB_COLOR);
    aSource->SetPixelType(VTK_UNSIGNED_CHAR);
    aSource->SetNumberOfScalarComponents(3);
    aSource->SetInputFrameSize(this->UndistortedFrame->cols, this->UndistortedFrame->rows, 1);
  }

  // Add the frame to the stream buffer
  FrameSizeType frameSize = { static_cast<unsigned int>(this->UndistortedFrame->cols), static_cast<unsigned int>(this->UndistortedFrame->rows), 1 };
  if (aSource->AddItem(this->UndistortedFrame->data, aSource->GetInputImageOrientation(), frameSize, VTK_UNSIGNED_CHAR, 3, US_IMG_RGB_COLOR, 0, this->FrameNumber) == PLUS_FAIL)
  {
    return PLUS_FAIL;
  }
  */
  this->FrameNumber++;

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusModusScraperTextSource::NotifyConfigured()
{
  return PLUS_SUCCESS;
}
