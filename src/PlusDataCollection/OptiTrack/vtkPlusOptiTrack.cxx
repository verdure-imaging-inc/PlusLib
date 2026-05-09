/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

// Local includes
#include "PlusConfigure.h"
#include "vtkPlusOptiTrack.h"
#include "vtkPlusDataSource.h"

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <vtkMath.h>
#include <vtkXMLDataElement.h>
#include <vtksys/Encoding.hxx>

// Motive API - dynamic loader replaces static linkage and version #ifdefs
#include "MotiveDynamicLoader.h"

// std includes
#include <set>

vtkStandardNewMacro(vtkPlusOptiTrack);

//----------------------------------------------------------------------------
class vtkPlusOptiTrack::vtkInternal
{
public:
  vtkPlusOptiTrack* External;

  vtkInternal(vtkPlusOptiTrack* external)
    : External(external)
    , NNClient(nullptr)
    , UnitsToMm(1.0)
    , MotiveDataDescriptionsUpdateTimeSec(1.0)
    , LastMotiveDataDescriptionsUpdateTimestamp(-1)
    , AttachToRunningMotive(-1)  // default: auto-detect
    , MotiveSkipped(false)
  {
  }

  virtual ~vtkInternal()
  {
  }

  // NatNet client, parameters, and callback function
  NatNetClient* NNClient;
  float UnitsToMm;

  // Motive Files (Motive 2.x+ use Profile + Calibration)
  std::string Profile;
  std::string Calibration;

  std::string CalibrationFile;
  std::vector<std::string> AdditionalRigidBodyFiles;

  // Maps rigid body names to transform names
  std::map<int, igsioTransformName> MapRBNameToTransform;

  // Flag to run Motive in background if user doesn't need GUI
  // -1 = auto-detect (check if Motive.exe is running)
  //  0 = false (start Motive API in background)
  //  1 = true (attach to running Motive via NatNet)
  int AttachToRunningMotive;

  // Flag: auto-detect determined Motive is not running, device is idle
  bool MotiveSkipped;

  // Time of last tool update
  double LastMotiveDataDescriptionsUpdateTimestamp;
  double MotiveDataDescriptionsUpdateTimeSec;

  /*!
  Receive updated tracking information from the server and push the new transforms to the tools
  */
  static void InternalCallback(sFrameOfMocapData* data, void* pUserData);

  void UpdateMotiveDataDescriptions();
};

//-----------------------------------------------------------------------
void vtkPlusOptiTrack::vtkInternal::UpdateMotiveDataDescriptions()
{
  LOG_TRACE("vtkPlusOptiTrack::vtkInternal::MatchTrackedTools");

  std::string referenceFrame = this->External->GetToolReferenceFrameName();
  this->MapRBNameToTransform.clear();
  sDataDescriptions* dataDescriptions;
  this->NNClient->GetDataDescriptions(&dataDescriptions);
  for (int i = 0; i < dataDescriptions->nDataDescriptions; ++i)
  {
    sDataDescription currentDescription = dataDescriptions->arrDataDescriptions[i];
    if (currentDescription.type == Descriptor_RigidBody)
    {
      // Map the numerical ID of the tracked tool from motive to the name of the tool
      igsioTransformName toolToTracker = igsioTransformName(currentDescription.Data.RigidBodyDescription->szName, referenceFrame);
      this->MapRBNameToTransform[currentDescription.Data.RigidBodyDescription->ID] = toolToTracker;
    }
  }

  this->LastMotiveDataDescriptionsUpdateTimestamp = vtkIGSIOAccurateTimer::GetSystemTime();
}

//-----------------------------------------------------------------------
vtkPlusOptiTrack::vtkPlusOptiTrack()
  : vtkPlusDevice()
  , Internal(new vtkInternal(this))
{
  this->FrameNumber = 0;
  // always uses NatNet's callback to update
  this->InternalUpdateRate = 120;
  this->StartThreadForInternalUpdates = false;
}

//----------------------------------------------------------------------------
vtkPlusOptiTrack::~vtkPlusOptiTrack()
{
  delete Internal;
  Internal = nullptr;
}

//----------------------------------------------------------------------------
void vtkPlusOptiTrack::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOptiTrack::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusOptiTrack::ReadConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Profile, this->Internal->Profile, deviceConfig);
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Calibration, this->Internal->Calibration, deviceConfig);

  // AttachToRunningMotive: TRUE/FALSE or -1 (auto-detect), 0 (false), 1 (true)
  const char* attachAttr = deviceConfig->GetAttribute("AttachToRunningMotive");
  if (attachAttr != nullptr)
  {
    std::string val(attachAttr);
    if (val == "TRUE" || val == "true" || val == "1")
    {
      this->Internal->AttachToRunningMotive = 1;
    }
    else if (val == "FALSE" || val == "false" || val == "0")
    {
      this->Internal->AttachToRunningMotive = 0;
    }
    else if (val == "-1" || val == "AUTO" || val == "auto")
    {
      this->Internal->AttachToRunningMotive = -1;
    }
    else
    {
      LOG_WARNING("Unknown AttachToRunningMotive value '" << val << "', using auto-detect (-1)");
      this->Internal->AttachToRunningMotive = -1;
    }
  }
  // else remains -1 (auto-detect)
  XML_READ_SCALAR_ATTRIBUTE_NONMEMBER_OPTIONAL(double, MotiveDataDescriptionsUpdateTimeSec, this->Internal->MotiveDataDescriptionsUpdateTimeSec, deviceConfig);

  XML_FIND_NESTED_ELEMENT_REQUIRED(dataSourcesElement, deviceConfig, "DataSources");
  for (int nestedElementIndex = 0; nestedElementIndex < dataSourcesElement->GetNumberOfNestedElements(); nestedElementIndex++)
  {
    vtkXMLDataElement* toolDataElement = dataSourcesElement->GetNestedElement(nestedElementIndex);
    if (STRCASECMP(toolDataElement->GetName(), "DataSource") != 0)
    {
      // if this is not a data source element, skip it
      continue;
    }
    if (toolDataElement->GetAttribute("Type") != NULL && STRCASECMP(toolDataElement->GetAttribute("Type"), "Tool") != 0)
    {
      // if this is not a Tool element, skip it
      continue;
    }

    std::string toolId(toolDataElement->GetAttribute("Id"));
    if (toolId.empty())
    {
      // tool doesn't have ID needed to generate transform
      LOG_ERROR("Failed to initialize OptiTrack tool: DataSource Id is missing. This should be the name of the Motive Rigid Body that tracks the tool.");
      continue;
    }

    if (toolDataElement->GetAttribute("RigidBodyFile") != NULL)
    {
      // this tool has an associated rigid body definition
      const char* rigidBodyFile = toolDataElement->GetAttribute("RigidBodyFile");
      this->Internal->AdditionalRigidBodyFiles.push_back(rigidBodyFile);
    }
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOptiTrack::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusOptiTrack::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOptiTrack::Probe()
{
  LOG_TRACE("vtkPlusOptiTrack::Probe");
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------
PlusStatus vtkPlusOptiTrack::InternalConnect()
{
  LOG_TRACE("vtkPlusOptiTrack::InternalConnect");

  // Auto-detect: check if Motive is already running
  bool attachToRunning = false;
  if (this->Internal->AttachToRunningMotive < 0)
  {
    attachToRunning = MotiveDynLoader::IsMotiveRunning();
    LOG_INFO("Motive auto-detect: Motive.exe is " << (attachToRunning ? "running, will attach" : "not running, skipping OptiTrack"));
    if (!attachToRunning)
    {
      this->Internal->MotiveSkipped = true;
      // Enable the internal update thread to push identity transforms periodically
      this->StartThreadForInternalUpdates = true;
      this->InternalUpdateRate = 30; // 30 Hz is enough for timestamp synchronization
      LOG_INFO("OptiTrack: skipped (Motive not running). Tracker tools initialized with identity transforms.");
      return PLUS_SUCCESS;
    }
  }
  else
  {
    attachToRunning = (this->Internal->AttachToRunningMotive != 0);
  }

  if (!attachToRunning)
  {
    // Load Motive API DLL at runtime
    if (!MotiveDynLoader::Load())
    {
      LOG_ERROR("Failed to load Motive API: " << MotiveDynLoader::GetLastError());
      return PLUS_FAIL;
    }
    LOG_INFO("Motive API loaded (version: " << (MotiveDynLoader::GetVersion() == MotiveVersion::V3_1 ? "3.1+" : "3.0.x") << ")");

    // Check no other Motive instance is consuming devices
    if (!MotiveDynLoader::CanConnectToDevices())
    {
      LOG_ERROR("Failed to start Motive. Another instance is already running.");
      return PLUS_FAIL;
    }

    // RUN MOTIVE IN BACKGROUND
    if (MotiveDynLoader::Initialize() != 0)
    {
      LOG_ERROR("Failed to start Motive.");
      return PLUS_FAIL;
    }

    // pick up recently-arrived cameras
    MotiveDynLoader::Update();

    // Load profile
    std::string profilePath = vtkPlusConfig::GetInstance()->GetDeviceSetConfigurationPath(this->Internal->Profile);
    std::wstring wProfilePath = vtksys::Encoding::ToWide(profilePath);
    if (MotiveDynLoader::LoadProfile(wProfilePath.c_str()) != 0)
    {
      LOG_ERROR("Failed to load Motive profile from: " << profilePath);
      return PLUS_FAIL;
    }

    // Load calibration
    std::string calibrationPath = vtkPlusConfig::GetInstance()->GetDeviceSetConfigurationPath(this->Internal->Calibration);
    std::wstring wCalibrationPath = vtksys::Encoding::ToWide(calibrationPath);
    if (MotiveDynLoader::LoadCalibration(wCalibrationPath.c_str()) != 0)
    {
      LOG_ERROR("Failed to load Motive calibration from: " << calibrationPath);
      return PLUS_FAIL;
    }

    // Enable NatNet streaming (equivalent to "Broadcast Frame Data" in Motive GUI)
    if (MotiveDynLoader::StreamNP(true) != 0)
    {
      LOG_ERROR("Failed to enable NatNet streaming.");
      return PLUS_FAIL;
    }

    // Add any additional rigid body files
    for (auto it = this->Internal->AdditionalRigidBodyFiles.begin(); it != this->Internal->AdditionalRigidBodyFiles.end(); it++)
    {
      std::string rbFilePath = vtkPlusConfig::GetInstance()->GetDeviceSetConfigurationPath(*it);
      std::wstring wRBFilePath = vtksys::Encoding::ToWide(rbFilePath);
      if (MotiveDynLoader::AddRigidBodies(wRBFilePath.c_str()) != 0)
      {
        LOG_ERROR("Failed to load rigid body file: " << rbFilePath);
        return PLUS_FAIL;
      }
    }

    LOG_INFO("\n---------------------------------MOTIVE SETTINGS--------------------------------");
    LOG_INFO("Connected cameras:");
    for (int i = 0; i < MotiveDynLoader::CameraCount(); i++)
    {
      wchar_t cameraName[256] = {};
      MotiveDynLoader::CameraName(i, cameraName, 256);
      LOG_INFO_W(i << L": " << cameraName);
    }
    LOG_INFO("\nUsing Motive profile located at:\n" << profilePath);
    LOG_INFO("\nUsing Motive calibration located at:\n" << calibrationPath);
    LOG_INFO("\nTracked rigid bodies:");
    for (int i = 0; i < MotiveDynLoader::RigidBodyCount(); ++i)
    {
      wchar_t rigidBodyName[256] = {};
      MotiveDynLoader::RigidBodyName(i, rigidBodyName, 256);
      LOG_INFO_W(rigidBodyName);
    }
    LOG_INFO("--------------------------------------------------------------------------------\n");

    this->StartThreadForInternalUpdates = true;
  }

  // CONFIGURE NATNET CLIENT
  this->Internal->NNClient = new NatNetClient(ConnectionType_Multicast);
  this->Internal->NNClient->SetVerbosityLevel(Verbosity_None);
  this->Internal->NNClient->SetVerbosityLevel(Verbosity_Warning);
  this->Internal->NNClient->SetDataCallback(vtkPlusOptiTrack::vtkInternal::InternalCallback, this);

  int retCode = this->Internal->NNClient->Initialize("127.0.0.1", "127.0.0.1");

  void* response;
  int nBytes;
  if (this->Internal->NNClient->SendMessageAndWait("UnitsToMillimeters", &response, &nBytes) == ErrorCode_OK)
  {
    this->Internal->UnitsToMm = (*(float*)response);
  }
  else
  {
    // Fail if motive is not running
    LOG_ERROR("Failed to connect to Motive. Please either set AttachToRunningMotive=FALSE or ensure that Motive is running and streaming is enabled.");
    return PLUS_FAIL;
  }

  // verify all rigid bodies in Motive have unique names
  std::set<std::string> rigidBodies;
  sDataDescriptions* dataDescriptions;
  this->Internal->NNClient->GetDataDescriptions(&dataDescriptions);
  for (int i = 0; i < dataDescriptions->nDataDescriptions; ++i)
  {
    sDataDescription currentDescription = dataDescriptions->arrDataDescriptions[i];
    if (currentDescription.type == Descriptor_RigidBody)
    {
      // Map the numerical ID of the tracked tool from motive to the name of the tool
      if (!rigidBodies.insert(currentDescription.Data.RigidBodyDescription->szName).second)
      {
        LOG_ERROR("Duplicate rigid bodies with name: " << currentDescription.Data.RigidBodyDescription->szName);
        return PLUS_FAIL;
      }
    }
  }

  // cause update of tools from Motive
  this->Internal->LastMotiveDataDescriptionsUpdateTimestamp = -1;

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------
PlusStatus vtkPlusOptiTrack::InternalDisconnect()
{
  LOG_TRACE("vtkPlusOptiTrack::InternalDisconnect");
  if (MotiveDynLoader::IsLoaded())
  {
    MotiveDynLoader::Shutdown();
    MotiveDynLoader::Unload();
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOptiTrack::InternalStartRecording()
{
  LOG_TRACE("vtkPlusOptiTrack::InternalStartRecording");
  if (this->Internal->MotiveSkipped) { return PLUS_SUCCESS; }
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOptiTrack::InternalStopRecording()
{
  if (this->Internal->MotiveSkipped) { return PLUS_SUCCESS; }
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOptiTrack::InternalUpdate()
{
  LOG_TRACE("vtkPlusOptiTrack::InternalUpdate");

  if (this->Internal->MotiveSkipped)
  {
    // Push identity transforms with current timestamp so output channels stay synchronized
    vtkSmartPointer<vtkMatrix4x4> identity = vtkSmartPointer<vtkMatrix4x4>::New();
    identity->Identity();
    const double timestamp = vtkIGSIOAccurateTimer::GetSystemTime();
    for (DataSourceContainerConstIterator it = this->GetToolIteratorBegin(); it != this->GetToolIteratorEnd(); ++it)
    {
      this->ToolTimeStampedUpdate(it->second->GetId(), identity, TOOL_OUT_OF_VIEW, this->FrameNumber, timestamp);
    }
    this->FrameNumber++;
    return PLUS_SUCCESS;
  }

  // InternalUpdate is only called if using Motive API (not attach mode)
  if (MotiveDynLoader::IsLoaded())
  {
    MotiveDynLoader::Update();
  }
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------
void vtkPlusOptiTrack::vtkInternal::InternalCallback(sFrameOfMocapData* data, void* pUserData)
{
  vtkPlusOptiTrack* self = (vtkPlusOptiTrack*)pUserData;

  LOG_TRACE("vtkPlusOptiTrack::InternalCallback");
  const double unfilteredTimestamp = vtkIGSIOAccurateTimer::GetSystemTime();

  if (self->Internal->LastMotiveDataDescriptionsUpdateTimestamp < 0)
  {
    // do an initial match of tracked tools
    self->Internal->UpdateMotiveDataDescriptions();
  }

  if (self->Internal->AttachToRunningMotive && self->Internal->MotiveDataDescriptionsUpdateTimeSec >= 0)
  {
    double timeSinceMotiveDataDescriptionsUpdate = unfilteredTimestamp - self->Internal->LastMotiveDataDescriptionsUpdateTimestamp;
    if (timeSinceMotiveDataDescriptionsUpdate > self->Internal->MotiveDataDescriptionsUpdateTimeSec)
    {
      self->Internal->UpdateMotiveDataDescriptions();
    }
  }

  int numberOfRigidBodies = data->nRigidBodies;
  sRigidBodyData* rigidBodies = data->RigidBodies;

  // identity transform for tools out of view
  vtkSmartPointer<vtkMatrix4x4> rigidBodyToTrackerMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

  for (int rigidBodyId = 0; rigidBodyId < numberOfRigidBodies; ++rigidBodyId)
  {
    // TOOL IN VIEW
    rigidBodyToTrackerMatrix->Identity();
    sRigidBodyData currentRigidBody = rigidBodies[rigidBodyId];

    if (currentRigidBody.MeanError != 0)
    {
      // convert translation to mm
      double translation[3] = { currentRigidBody.x * self->Internal->UnitsToMm, currentRigidBody.y * self->Internal->UnitsToMm, currentRigidBody.z * self->Internal->UnitsToMm };

      // convert rotation from quaternion to 3x3 matrix
      double quaternion[4] = { currentRigidBody.qw, currentRigidBody.qx, currentRigidBody.qy, currentRigidBody.qz };
      double rotation[3][3] = { 0,0,0, 0,0,0, 0,0,0 };
      vtkMath::QuaternionToMatrix3x3(quaternion, rotation);

      // construct the transformation matrix from the rotation and translation components
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          rigidBodyToTrackerMatrix->SetElement(i, j, rotation[i][j]);
        }
        rigidBodyToTrackerMatrix->SetElement(i, 3, translation[i]);
      }

      // check if tool is in view
      bool bTrackingValid = currentRigidBody.params & 0x01;

      // make sure the tool was specified in the Config file
      igsioTransformName toolToTracker = self->Internal->MapRBNameToTransform[currentRigidBody.ID];
      self->ToolTimeStampedUpdate(toolToTracker.GetTransformName(), rigidBodyToTrackerMatrix, (bTrackingValid ? TOOL_OK : TOOL_INVALID), self->FrameNumber, unfilteredTimestamp);
    }
    else
    {
      // TOOL OUT OF VIEW
      igsioTransformName toolToTracker = self->Internal->MapRBNameToTransform[currentRigidBody.ID];
      self->ToolTimeStampedUpdate(toolToTracker.GetTransformName(), rigidBodyToTrackerMatrix, TOOL_OUT_OF_VIEW, self->FrameNumber, unfilteredTimestamp);
    }

  }

  self->FrameNumber++;
}
