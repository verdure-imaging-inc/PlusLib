/*=Plus=header=begin======================================================
    Program: Plus
    Copyright (c) Verdure Imaging Inc, Stockton, California. All rights reserved.
    See License.txt for details.

    We would like to acknowledge Verdure Imaging Inc for generously open-sourcing
    this support for the Clarius OEM interface to the PLUS & Slicer communities.
=========================================================Plus=header=end*/

// Local includes
#include "PlusConfigure.h"
#include "vtkPlusChannel.h"
#include "vtkPlusClariusOEM.h"
#include "PixelCodec.h"
#include "vtkPlusDataSource.h"
#include "vtkPlusUsImagingParameters.h"
#include "ClariusWifiHelper.h"

// VTK includes
#include <vtkImageData.h>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// STL includes
#include <algorithm>
#include <cctype>
#include <chrono>
#include <future>
#include <map>
#include <sstream>
#include <string>

// Clarius bluetooth helper includes
#include <CBTInterface.h>

// Clarius Includes
#include <oem.h>
#include <oem_def.h>

// Clarius enable / disable values
#define CLARIUS_TRUE 1
#define CLARIUS_FALSE 0
#define CLARIUS_RUN 1
#define CLARIUS_STOP 0

// Clarius connection state values
#define CLARIUS_STATE_NOT_INITIALIZED -1
#define CLARIUS_STATE_NOT_CONNECTED 0
#define CLARIUS_STATE_CONNECTED 1

namespace
{
  static const double CM_TO_MM = 10.0;
  static const double MM_TO_CM = 0.1;

  static const int CLARIUS_SHORT_DELAY_MS = 10;
  static const int CLARIUS_LONG_DELAY_MS = 2000;

  static const FrameSizeType DEFAULT_FRAME_SIZE = { 512, 512, 1 };

  static const bool DEFAULT_ENABLE_AUTO_GAIN = false;

  static const bool DEFAULT_ENABLE_IMU = false;

  static const bool DEFAULT_ENABLE_BUTTONS = false;

  static const bool DEFAULT_ENABLE_5V_RAIL = false;

  static const double DEFAULT_DEPTH_MM = 100.0;

  static const double DEFAULT_GAIN_PERCENT = 80.0;

  static const double DEFAULT_DYNRANGE_PERCENT = 80.0;

  static const std::vector<double> DEFAULT_TGC = { 0.5, 0.5, 0.5 };

  static std::map<int, std::string> ConnectEnumToString {
    {CONNECT_SUCCESS, "CONNECT_SUCCESS"},
    {CONNECT_DISCONNECT, "CONNECT_DISCONNECT"},
    {CONNECT_FAILED, "CONNECT_FAILED"},
    {CONNECT_SWUPDATE, "CONNECT_SWUPDATE"},
    {CONNECT_ERROR, "CONNECT_ERROR"},
  };

}

//-------------------------------------------------------------------------------------------------

vtkPlusClariusOEM* vtkPlusClariusOEM::instance;

//-------------------------------------------------------------------------------------------------
// vtkInternal
//-------------------------------------------------------------------------------------------------

class vtkPlusClariusOEM::vtkInternal
{
public:

  vtkInternal(vtkPlusClariusOEM* ext);

  virtual ~vtkInternal()
  {
  }

protected:

  friend class vtkPlusClariusOEM;

  // Clarius callbacks
  static void ListFn(const char* list, int sz);

  static void ConnectFn(int ret, int port, const char* status);

  static void CertFn(int daysValid);

  static void PowerDownFn(int ret, int tm);

  static void SwUpdateFn(int ret);

  static void RawImageFn(const void* newImage, const ClariusRawImageInfo* nfo, int npos, const ClariusPosInfo* pos);

  static void ProcessedImageFn(const void* newImage, const ClariusProcessedImageInfo* nfo, int npos, const ClariusPosInfo* pos);

  static void SpectralImageFn(const void* newImage, const ClariusSpectralImageInfo* nfo);

  static void ImagingFn(int ready, int imaging);

  static void ButtonFn(int btn, int clicks);

  static void ProgressFn(int progress);

  static void ErrorFn(const char* msg);
  
  // user configurable params
  std::string ProbeSerialNum;
  std::string PathToCert;
  FrameSizeType FrameSize;
  std::string ProbeType;
  std::string ImagingApplication;
  bool EnableAutoGain;
  bool EnableImu;
  bool EnableButtons;
  bool Enable5vRail;

  // parameters retrieved from the probe over BLE
  std::string Ssid;
  std::string Password;
  std::string IpAddress;
  int TcpPort;

  // parameters retrieved from the probe using the OEM API
  std::promise<std::vector<std::string>> PromiseProbes;
  std::promise<std::vector<std::string>> PromiseApplications;
  std::promise<void> ConnectionBarrier;

  enum class EXPECTED_LIST
  {
    PROBES,
    APPLICATIONS,
    UNKNOWN
  } ExpectedList;

private:
  vtkPlusClariusOEM* External;

  // BLE interface
  CBTInterface* BluetoothInterface;

  // Wifi helper class
  ClariusWifiHelper WifiHelper;
};

//-------------------------------------------------------------------------------------------------
vtkPlusClariusOEM::vtkInternal::vtkInternal(vtkPlusClariusOEM* ext)
: External(ext)
, ProbeSerialNum("")
, PathToCert("")
, FrameSize(DEFAULT_FRAME_SIZE)
, ProbeType("")
, ImagingApplication("")
, EnableAutoGain(DEFAULT_ENABLE_AUTO_GAIN)
, EnableImu(DEFAULT_ENABLE_IMU)
, EnableButtons(DEFAULT_ENABLE_BUTTONS)
, Enable5vRail(DEFAULT_ENABLE_5V_RAIL)
, IpAddress("")
, TcpPort(-1)
, BluetoothInterface(nullptr)
{
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::ListFn(const char* list, int sz)
{
  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
  
  std::vector<std::string> vec;

  std::stringstream ss(list);
  while (ss.good())
  {
    std::string substr;
    getline(ss, substr, ',');
    vec.push_back(substr);
  }

  if (device->Internal->ExpectedList == EXPECTED_LIST::PROBES)
  {
    device->Internal->PromiseProbes.set_value(vec);
  }
  else if (device->Internal->ExpectedList == EXPECTED_LIST::APPLICATIONS)
  {
    device->Internal->PromiseApplications.set_value(vec);
  }
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::ConnectFn(int ret, int port, const char* status)
{
  if (ret == CONNECT_SUCCESS)
  {
    // connection succeeded, set Internal->Connected variable to end busy wait in InternalConnect
    vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
    device->Internal->ConnectionBarrier.set_value();
  }
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::CertFn(int daysValid)
{
  LOG_INFO("CertFn: days_valid=" << daysValid);
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::PowerDownFn(int ret, int tm)
{
  LOG_INFO("PowerDownFn: ret=" << ret << " tm=" << tm);
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::SwUpdateFn(int ret)
{
  LOG_INFO("SwUpdateFn: ret=" << ret);
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::RawImageFn(const void* newImage, const ClariusRawImageInfo* nfo, int npos, const ClariusPosInfo* pos)
{
  LOG_INFO("RawImageFn");
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::SpectralImageFn(const void* newImage, const ClariusSpectralImageInfo* nfo)
{
  LOG_INFO("SpectralImageFn");
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::ProcessedImageFn(const void* newImage, const ClariusProcessedImageInfo* nfo, int npos, const ClariusPosInfo* pos)
{
  auto ts = std::chrono::high_resolution_clock::now();

  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
  if (device == NULL)
  {
    LOG_ERROR("Clarius instance is NULL!!!");
    return;
  }

  // Check if still connected
  if (device->Connected == 0)
  {
    LOG_ERROR("Trouble connecting to Clarius Device. IpAddress = " << device->Internal->IpAddress
      << " port = " << device->Internal->TcpPort);
    return;
  }

  if (newImage == NULL)
  {
    LOG_ERROR("No frame received by the device");
    return;
  }

  // check if there exist active data source;
  vtkPlusDataSource* bModeSource;
  std::vector<vtkPlusDataSource*> bModeSources;
  device->GetVideoSourcesByPortName(vtkPlusDevice::BMODE_PORT_NAME, bModeSources);
  if (!bModeSources.empty())
  {
    bModeSource = bModeSources[0];
  }
  else
  {
    LOG_WARNING("Processed image was received, however no output B-Mode video source was found.");
    return;
  }

  // Set Image Properties
  bModeSource->SetInputFrameSize(nfo->width, nfo->height, 1);
  bModeSource->SetNumberOfScalarComponents(1);

  // convert RGBA to grayscale
  // format is:
  // R = ultrasound value
  // G = ultrasound value
  // B = ultrasound value
  // A = 255
  std::vector<unsigned char> _gray_img;
  _gray_img.resize(nfo->imageSize / 4);
  PixelCodec::ConvertToGray(
    PixelCodec::PixelEncoding_RGBA32,
    nfo->width,
    nfo->height,
    (unsigned char*) newImage,
    _gray_img.data()
  );

  const double unfilteredTimestamp = vtkIGSIOAccurateTimer::GetSystemTime();

  igsioFieldMapType customField;
  customField["micronsPerPixel"] = std::make_pair(igsioFrameFieldFlags::FRAMEFIELD_FORCE_SERVER_SEND, std::to_string(nfo->micronsPerPixel));
  bModeSource->AddItem(
    _gray_img.data(), // pointer to char array
    bModeSource->GetInputImageOrientation(), // refer to this url: http://perk-software.cs.queensu.ca/plus/doc/nightly/dev/UltrasoundImageOrientation.html for reference;
                                         // Set to UN to keep the orientation of the image the same as on tablet
    bModeSource->GetInputFrameSize(),
    VTK_UNSIGNED_CHAR,
    1,
    US_IMG_BRIGHTNESS,
    0,
    device->FrameNumber,
    unfilteredTimestamp,
    unfilteredTimestamp,
    &customField
  );

  auto tf = std::chrono::high_resolution_clock::now();

  auto us = std::chrono::duration_cast<std::chrono::microseconds>(tf - ts);
  LOG_INFO("ProcImFn elapsed: " << us.count());
  device->FrameNumber++;
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::ImagingFn(int ready, int imaging)
{
  LOG_INFO("ImagingFn: ready=" << ready << " imaging: " << imaging);

  if (ready == IMAGING_READY)
  {
    LOG_INFO("ready to image: " << ((imaging) ? "imaging running" : "imaging stopped"));
  }
  else if (ready == IMAGING_CERTEXPIRED)
  {
    LOG_ERROR("certificate needs updating prior to imaging");
  }
  else
  {
    LOG_ERROR("not ready to image");
  }
}

//-------------------------------------------------------------------------------------------------
/*! callback for button clicks
 * @param[in] btn 0 = up, 1 = down
 * @param[in] clicks # of clicks performed*/
void vtkPlusClariusOEM::vtkInternal::ButtonFn(int btn, int clicks)
{
  LOG_INFO("button: " << btn << "clicks: " << clicks << "%");
}

//-------------------------------------------------------------------------------------------------
/*! callback for readback progress
 * @pram[in] progress the readback process*/
void vtkPlusClariusOEM::vtkInternal::ProgressFn(int progress)
{
  LOG_INFO("Download: " << progress << "%");
}


//-------------------------------------------------------------------------------------------------
/*! callback for error messages
 * @param[in] err the error message sent from the listener module
 * */
void vtkPlusClariusOEM::vtkInternal::ErrorFn(const char* err)
{
  LOG_ERROR("error: " << err);
}

//-------------------------------------------------------------------------------------------------
// vtkPlusClariusOEM definitions
//-------------------------------------------------------------------------------------------------
vtkPlusClariusOEM* vtkPlusClariusOEM::New()
{
  if (instance == NULL)
  {
    instance = new vtkPlusClariusOEM();
  }
  return instance;
}

//-------------------------------------------------------------------------------------------------
vtkPlusClariusOEM::vtkPlusClariusOEM()
  : Internal(new vtkInternal(this))
{
  this->StartThreadForInternalUpdates = false;
  this->RequirePortNameInDeviceSetConfiguration = true;

  this->ImagingParameters->SetDepthMm(DEFAULT_DEPTH_MM);
  this->ImagingParameters->SetGainPercent(DEFAULT_GAIN_PERCENT);
  this->ImagingParameters->SetDynRangeDb(DEFAULT_DYNRANGE_PERCENT);
  
  this->ImagingParameters->SetTimeGainCompensation(DEFAULT_TGC);

  instance = this;
}

//-------------------------------------------------------------------------------------------------
vtkPlusClariusOEM::~vtkPlusClariusOEM()
{
  // ensure resources released
  this->InternalDisconnect();

  if (this->Internal)
  {
    delete this->Internal;
    this->Internal = nullptr;
  }

  this->instance = NULL;
}

//-------------------------------------------------------------------------------------------------
vtkPlusClariusOEM* vtkPlusClariusOEM::GetInstance()
{
  if (instance != NULL)
  {
    return instance;
  }
  else
  {
    // Instance is null, creating new instance
    instance = new vtkPlusClariusOEM();
    return instance;
  }
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::PrintSelf(ostream& os, vtkIndent indent)
{
  LOG_TRACE("vtkPlusClariusOEM::PrintSelf");

  this->Superclass::PrintSelf(os, indent);
  os << indent << "ipAddress" << this->Internal->IpAddress << std::endl;
  os << indent << "tcpPort" << this->Internal->TcpPort << std::endl;
  os << indent << "FrameNumber" << this->FrameNumber << std::endl;
  FrameSizeType fs = this->Internal->FrameSize;
  os << indent << "FrameWidth" << fs[0] << std::endl;
  os << indent << "FrameHeight" << fs[1] << std::endl;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::ParseImuConfig(vtkXMLDataElement* deviceConfig)
{
  LOG_TRACE("vtkPlusClariusOEM::ParseImuConfig");

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusClariusOEM::ReadConfiguration");

  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  // probe serial number
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(
    ProbeSerialNum, this->Internal->ProbeSerialNum, deviceConfig);

  // path to Clarius certificate
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(
    PathToCert, this->Internal->PathToCert, deviceConfig);
  
  // probe type
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(
    ProbeType, this->Internal->ProbeType, deviceConfig);
  // force probe type string to be entirely uppercase
  std::transform(
    this->Internal->ProbeType.begin(),
    this->Internal->ProbeType.end(),
    this->Internal->ProbeType.begin(),
    [](unsigned char c) { return std::toupper(c); }
  );

  // imaging application (msk, abdomen, etc.)
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(
    ImagingApplication, this->Internal->ImagingApplication, deviceConfig);
  // force imaging application string to be entirely lowercase
  std::transform(
    this->Internal->ImagingApplication.begin(),
    this->Internal->ImagingApplication.end(),
    this->Internal->ImagingApplication.begin(),
    [](unsigned char c) { return std::tolower(c); }
  );

  // frame size
  int rfs[2] = { static_cast<int>(DEFAULT_FRAME_SIZE[0]), static_cast<int>(DEFAULT_FRAME_SIZE[1]) };
  if (deviceConfig->GetVectorAttribute("FrameSize", 2, rfs))
  {
    if (rfs[0] < 0 || rfs[1] < 0)
    {
      LOG_ERROR("Negative frame size defined in config file. Please define a positive frame size.");
      return PLUS_FAIL;
    }
    FrameSizeType fs = { static_cast<unsigned int>(rfs[0]), static_cast<unsigned int>(rfs[1]), 1 };
    this->Internal->FrameSize = fs;
  }

  // enable auto gain
  XML_READ_BOOL_ATTRIBUTE_NONMEMBER_OPTIONAL(EnableAutoGain,
    this->Internal->EnableAutoGain, deviceConfig);

  // enable IMU
  XML_READ_BOOL_ATTRIBUTE_NONMEMBER_OPTIONAL(EnableImu,
    this->Internal->EnableImu, deviceConfig);

  // enable button presses
  XML_READ_BOOL_ATTRIBUTE_NONMEMBER_OPTIONAL(EnableButtons,
    this->Internal->EnableButtons, deviceConfig);

  // enable 5v rail
  XML_READ_BOOL_ATTRIBUTE_NONMEMBER_OPTIONAL(Enable5v,
    this->Internal->Enable5vRail, deviceConfig);

  // read imaging parameters
  this->ImagingParameters->ReadConfiguration(deviceConfig);

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusClariusOEM::WriteConfiguration");

  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);

  this->ImagingParameters->WriteConfiguration(deviceConfig);

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::NotifyConfigured()
{
  LOG_TRACE("vtkPlusClariusOEM::NotifyConfigured");

  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
  if (device->OutputChannels.size() > 1)
  {
    LOG_WARNING("vtkPlusClariusOEM is expecting one output channel and there are " <<
      this->OutputChannels.size() << " channels. First output channel will be used.");
  }

  if (device->OutputChannels.empty())
  {
    LOG_ERROR("No output channels defined for vtkPlusClariusOEM. Cannot proceed.");
    this->CorrectlyConfigured = false;
    return PLUS_FAIL;
  }

  std::vector<vtkPlusDataSource*> sources;
  sources = device->GetVideoSources();
  if (sources.size() > 1)
  {
    LOG_WARNING("More than one output video source found. First will be used");
  }
  if (sources.size() == 0)
  {
    LOG_ERROR("Video source required in configuration. Cannot proceed.");
    return PLUS_FAIL;
  }

  // Check if output channel has data source
  vtkPlusDataSource* aSource(NULL);
  if (device->OutputChannels[0]->GetVideoSource(aSource) != PLUS_SUCCESS)
  {
    LOG_ERROR("Unable to retrieve the video source in the vtkPlusClariusOEM device.");
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::Probe()
{
  LOG_TRACE("vtkPlusClariusOEM: Probe");
 
  return PLUS_FAIL;
};

//-------------------------------------------------------------------------------------------------
std::string vtkPlusClariusOEM::GetSdkVersion()
{
  LOG_TRACE("vtkPlusClariusOEM::GetSdkVersion");

  std::ostringstream version;
  version << "Sdk version not available" << "\n";
  return version.str();
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InitializeBLE()
{
  // construct bluetooth interface
  if (!this->Internal->BluetoothInterface)
  {
    this->Internal->BluetoothInterface = new CBTInterface();
  }

  // initialize CBTInterface
  if (!this->Internal->BluetoothInterface->Initialize())
  {
    LOG_ERROR("An error occurred during BLE Initialize. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  // search for probes
  std::vector<std::string> probes;
  if (!this->Internal->BluetoothInterface->SearchForProbes(probes))
  {
    LOG_ERROR("An error occurred during SearchForProbes. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  // check at least one probe found
  if (probes.size() == 0)
  {
    LOG_ERROR("No Clarius probes found... Please check probe has battery, and is connected to Windows bluetooth");
    return PLUS_FAIL;
  }

  // check desired probe is available
  bool isProbeAvailable = false;
  for (std::string probe : probes)
  {
    if (igsioCommon::IsEqualInsensitive(probe, this->Internal->ProbeSerialNum))
    {
      isProbeAvailable = true;
    }
  }

  if (!isProbeAvailable)
  {
    LOG_ERROR("Desired Clarius probe with SN: " << this->Internal->ProbeSerialNum << " was not found");
    return PLUS_FAIL;
  }

  // attempt to connect to probe ble network
  bool connectionResult = false;
  int tries = 0;
  while (tries < 10)
  {
    if (!this->Internal->BluetoothInterface->ConnectToProbeBT(this->Internal->ProbeSerialNum, connectionResult))
    {
      LOG_WARNING("Failed to connect to Clarius probe on try: " << (tries + 1));
      tries++;
      continue;
    }

    if (!connectionResult)
    {
      LOG_WARNING("Failed to connect to Clarius probe on try: " << (tries + 1));
      tries++;
    }
    else
    {
      break;
    }
  }

  // if still not connected, hard fail
  if (!connectionResult)
  {
    LOG_ERROR("Failed to connect to Clarius probe after 10 tries");
    return PLUS_FAIL;
  }

  // verify probe is connected
  bool isConnected;
  if (!this->Internal->BluetoothInterface->IsProbeConnected(isConnected))
  {
    LOG_ERROR("Failed to verify connection to Clarius probe in InitializeBLE. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  if (!isConnected)
  {
    LOG_ERROR("IsProbeConnected returned FALSE, but probe connection indicated success, aborting from unexpected state");
    return PLUS_FAIL;
  }

  // BLE connection succeeded
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InitializeProbe()
{
  // power on the probe
  bool powerState;
  if (!this->Internal->BluetoothInterface->PowerProbe(true, powerState))
  {
    LOG_ERROR("An error occurred during PowerProbe. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  if (!powerState)
  {
    LOG_ERROR("Failed to power on Clarius probe");
    return PLUS_FAIL;
  }

  // wait for probe to fully power on
  std::this_thread::sleep_for(std::chrono::seconds(10));

  // configure wifi AP
  bool configWifiResult;
  if (!this->Internal->BluetoothInterface->ConfigureWifiAP(configWifiResult))
  {
    LOG_ERROR("An error occurred during ConfigureWifiAP. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  if (!configWifiResult)
  {
    LOG_ERROR("Failed to configure Clarius probe wifi");
    return PLUS_FAIL;
  }

  // wait for wifi info to be applied
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // get & print wifi info
  ClariusWifiInfo info;
  if (!this->Internal->BluetoothInterface->GetWifiInfo(info))
  {
    LOG_ERROR("An error occurred during GetWifiInfo. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  if (!info.IsConnected)
  {
    LOG_WARNING("Wifi was not configured when wifi info was requested")
  }
  else
  {
    LOG_INFO("Clarius Wifi Info: ");
    LOG_INFO("Mode: " << (info.IsWifiAP ? "AP" : "LAN"));
    LOG_INFO("SSID: " << info.SSID);
    LOG_INFO("Password: " << info.Password);
    LOG_INFO("IPv4: " << info.IPv4);
    LOG_INFO("Control Port: " << info.ControlPort);
    LOG_INFO("Cast Port: " << info.CastPort);

    this->Internal->Ssid = info.SSID;
    this->Internal->Password = info.Password;
    this->Internal->IpAddress = info.IPv4;
    this->Internal->TcpPort = info.ControlPort;
  }

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InitializeWifi()
{
  if (this->Internal->WifiHelper.Initialize() != PLUS_SUCCESS)
  {
    LOG_ERROR("Failed to initialize Clarius wifi helper");
    return PLUS_FAIL;
  }

  PlusStatus res = this->Internal->WifiHelper.ConnectToClariusWifi(
    this->Internal->Ssid,
    this->Internal->Password
  );
  if (res != PLUS_SUCCESS)
  {
    LOG_ERROR("Failed to connect to Clarius probe wifi");
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InitializeOEM()
{
  // placeholder argc / argv arguments
  int argc = 1;
  char** argv = new char* [1];
  argv[0] = new char[4];
  strcpy(argv[0], "abc");
  const char* certPath = "/Clarius";

  // api callback functions
  ClariusListFn listFnPtr = static_cast<ClariusListFn>(&vtkPlusClariusOEM::vtkInternal::ListFn);
  ClariusConnectFn connectFnPtr = static_cast<ClariusConnectFn>(&vtkPlusClariusOEM::vtkInternal::ConnectFn);
  ClariusCertFn certFnPtr = static_cast<ClariusCertFn>(&vtkPlusClariusOEM::vtkInternal::CertFn);
  ClariusPowerDownFn powerDownFnPtr = static_cast<ClariusPowerDownFn>(&vtkPlusClariusOEM::vtkInternal::PowerDownFn);
  ClariusSwUpdateFn swUpdateFnPtr = static_cast<ClariusSwUpdateFn>(&vtkPlusClariusOEM::vtkInternal::SwUpdateFn);
  ClariusNewRawImageFn newRawImageFnPtr = static_cast<ClariusNewRawImageFn>(&vtkPlusClariusOEM::vtkInternal::RawImageFn);
  ClariusNewProcessedImageFn newProcessedImageFnPtr = static_cast<ClariusNewProcessedImageFn>(&vtkPlusClariusOEM::vtkInternal::ProcessedImageFn);
  ClariusNewSpectralImageFn newSpectralImageFnPtr = static_cast<ClariusNewSpectralImageFn>(&vtkPlusClariusOEM::vtkInternal::SpectralImageFn);
  ClariusImagingFn imagingFnPtr = static_cast<ClariusImagingFn>(&vtkPlusClariusOEM::vtkInternal::ImagingFn);
  ClariusButtonFn buttonFnPtr = static_cast<ClariusButtonFn>(&vtkPlusClariusOEM::vtkInternal::ButtonFn);
  ClariusProgressFn progressFnPtr = static_cast<ClariusProgressFn>(&vtkPlusClariusOEM::vtkInternal::ProgressFn);
  ClariusErrorFn errorFnPtr = static_cast<ClariusErrorFn>(&vtkPlusClariusOEM::vtkInternal::ErrorFn);

  // no b-mode data sources, disable b mode callback
  std::vector<vtkPlusDataSource*> bModeSources;
  this->GetVideoSourcesByPortName(vtkPlusDevice::BMODE_PORT_NAME, bModeSources);
  if (bModeSources.empty())
  {
    newProcessedImageFnPtr = nullptr;
  }

  // no RF-mode data sources, disable RF-mode callback
  std::vector<vtkPlusDataSource*> rfModeSources;
  this->GetVideoSourcesByPortName(vtkPlusDevice::RFMODE_PORT_NAME, rfModeSources);
  if (rfModeSources.empty())
  {
    newRawImageFnPtr = nullptr;
  }

  try
  {
    FrameSizeType fs = this->Internal->FrameSize;

    LOG_INFO("fs: [" << fs[0] << ", " << fs[1] << "]");

    int result = cusOemInit(
      argc,
      argv,
      certPath,
      connectFnPtr,
      certFnPtr,
      powerDownFnPtr,
      newProcessedImageFnPtr,
      newRawImageFnPtr,
      newSpectralImageFnPtr,
      imagingFnPtr,
      buttonFnPtr,
      errorFnPtr,
      fs[0],
      fs[1]
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_LONG_DELAY_MS));

    if (result < 0)
    {
      LOG_ERROR("Failed to initialize Clarius OEM library");
      return PLUS_FAIL;
    }
  }
  catch (const std::runtime_error& re)
  {
    LOG_ERROR("Runtime error on cusOemInit. Error text: " << re.what());
    return PLUS_FAIL;
  }
  catch (const std::exception& ex)
  {
    LOG_ERROR("Exception on cusOemInit. Error text: " << ex.what());
    return PLUS_FAIL;
  }
  catch (...)
  {
    LOG_ERROR("Unknown failure occurred on cusOemInit");
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;

}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::SetClariusCert()
{
  // load the cert file
  std::string fullCertPath = vtkPlusConfig::GetInstance()->GetDeviceSetConfigurationPath(this->Internal->PathToCert);
  std::ifstream certFile(fullCertPath);
  if (!certFile.is_open())
  {
    LOG_ERROR("Failed to open Clarius cert file from " << fullCertPath << ". Please check the PathToCert path in your config.");
    return PLUS_FAIL;
  }

  std::ostringstream sstr;
  sstr << certFile.rdbuf();
  std::string certStr = sstr.str();

  // set cert in OEM API
  if (cusOemSetCert(certStr.c_str()) != 0)
  {
    LOG_ERROR("Failed to set Clarius OEM connection certificate");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_LONG_DELAY_MS));

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::ConfigureProbeApplication()
{
  const char* ip = this->Internal->IpAddress.c_str();
  unsigned int port = this->Internal->TcpPort;
  LOG_INFO("Attempting connection to Clarius ultrasound on " << ip << ":" << port << " for 10 seconds:");
  
  std::future<void> connectionBarrierFuture = this->Internal->ConnectionBarrier.get_future();
  try
  {
    int result = cusOemConnect(ip, port);
    if (result != CONNECT_SUCCESS)
    {
      LOG_ERROR("Failed to initiate connection to Clarius probe on " << ip << ":" << port <<
        ". Return code: " << ConnectEnumToString[result]);
      return PLUS_FAIL;
    }
  }
  catch (const std::runtime_error& re)
  {
    LOG_ERROR("Runtime error on cusOemConnect. Error text: " << re.what());
    return PLUS_FAIL;
  }
  catch (const std::exception& ex)
  {
    LOG_ERROR("Exception on cusOemConnect. Error text: " << ex.what());
    return PLUS_FAIL;
  }
  catch (...)
  {
    LOG_ERROR("Unknown failure occurred on cusOemConnect");
    return PLUS_FAIL;
  }

  // wait for cusOemConnected call to complete
  if (connectionBarrierFuture.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
  {
    LOG_ERROR("Connection to Clarius device timed out");
    return PLUS_FAIL;
  }
  LOG_INFO("Connected to Clarius probe on " << ip << ":" << port);

  // get list of available probes
  ClariusListFn listFnPtr = static_cast<ClariusListFn>(&vtkPlusClariusOEM::vtkInternal::ListFn);
  this->Internal->ExpectedList = vtkPlusClariusOEM::vtkInternal::EXPECTED_LIST::PROBES;
  std::future<std::vector<std::string>> futureProbes = this->Internal->PromiseProbes.get_future();
  if (cusOemProbes(listFnPtr) != 0)
  {
    LOG_INFO("Failed to retrieve list of valid probe types");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_LONG_DELAY_MS));

  // wait for probes list to be populated
  if (futureProbes.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    LOG_ERROR("Failed to retrieve list of valid Clarius probe names");
    return PLUS_FAIL;
  }
  std::vector<std::string> vProbes = futureProbes.get();

  // validate provided probe type
  std::string probeType = this->Internal->ProbeType;
  if (std::find(vProbes.begin(), vProbes.end(), probeType) == vProbes.end())
  {
    std::string vProbesStr;
    for (const auto& probe : vProbes)
    {
      vProbesStr += probe + ", ";
    }
    vProbesStr.pop_back(); vProbesStr.pop_back(); // remove trailing comma and space
    LOG_ERROR("Invalid probe type (" << probeType << ") provided, valid probe types are: " << vProbesStr);
    return PLUS_FAIL;
  }

  // list available imaging applications
  this->Internal->ExpectedList = vtkPlusClariusOEM::vtkInternal::EXPECTED_LIST::APPLICATIONS;
  std::future<std::vector<std::string>> futureApplications = this->Internal->PromiseApplications.get_future();
  if (cusOemApplications(probeType.c_str(), listFnPtr) != 0)
  {
    LOG_ERROR("Failed to retrieve list of valid imaging applications");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_LONG_DELAY_MS));

  // wait for applications list to be populated
  if (futureApplications.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    LOG_ERROR("Failed to retrieve list of valid Clarius application names");
    return PLUS_FAIL;
  }
  std::vector<std::string> vApps = futureApplications.get();

  // validate provided imaging application
  std::string imagingApplication = this->Internal->ImagingApplication;
  if (std::find(vApps.begin(), vApps.end(), imagingApplication) == vApps.end())
  {
    std::string vAppsStr;
    for (const auto& app : vApps)
    {
      vAppsStr += app + ", ";
    }
    vAppsStr.pop_back(); vAppsStr.pop_back(); // remove trailing comma and space
    LOG_ERROR("Invalid imaging application (" << imagingApplication << ") provided, valid imaging applications are: " << vAppsStr);
    return PLUS_FAIL;
  }

  // configure probe mode
  if (cusOemLoadApplication(probeType.c_str(), imagingApplication.c_str()) == 0)
  {
    LOG_INFO("Attempting to load " << imagingApplication << " application on a " << probeType << " probe");
  }
  else
  {
    LOG_ERROR("An error occured on call to cusOemLoadApplication");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_LONG_DELAY_MS));

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::SetInitialUsParams()
{
  // set imaging depth (mm)
  double depthMm = this->ImagingParameters->GetDepthMm();
  if (this->SetDepthMm(depthMm) != PLUS_SUCCESS)
  {
    LOG_WARNING("Failed to set requested imaging depth (mm) in Clarius OEM device, unknown depth will be used");
  }

  // set gain (%)
  double gainPercent = this->ImagingParameters->GetGainPercent();
  if (this->SetGainPercent(gainPercent) != PLUS_SUCCESS)
  {
    LOG_WARNING("Failed to set requested imaging gain (%) in Clarius OEM device, unknown gain will be used");
  }

  // set dynamic range (%)
  double dynRangePercent = this->ImagingParameters->GetDynRangeDb();
  if (this->SetDynRangePercent(dynRangePercent) != PLUS_SUCCESS)
  {
    LOG_WARNING("Failed to set requested imaging dynamic range in Clarius OEM device, unknown dynamic range will be used");
  }

  // set time gain compensation
  std::vector<double> tgcPercent = this->ImagingParameters->GetTimeGainCompensation();
  if (this->SetTimeGainCompensationPercent(tgcPercent) != PLUS_SUCCESS)
  {
    LOG_WARNING("Failed to set requested imaging time gain compensation in Clarius OEM device, unknown time gain compensation will be used");
  }

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalConnect()
{
  LOG_TRACE("vtkPlusClariusOEM::InternalConnect");

  if (this->Connected)
  {
    // Internal connect already called and completed successfully
    return PLUS_SUCCESS;
  }

  // BLE
  if (this->InitializeBLE() != PLUS_SUCCESS)
  {
    LOG_ERROR("Failed to initialize BLE in Clarius OEM device");
    this->InternalDisconnect();
    return PLUS_FAIL;
  }

  // PROBE (power, etc.)
  if (this->InitializeProbe() != PLUS_SUCCESS)
  {
    LOG_ERROR("Failed to initialize probe (power, wifi settings) in Clarius OEM device");
    this->InternalDisconnect();
    return PLUS_FAIL;
  }

  // WIFI
  if (this->InitializeWifi() != PLUS_SUCCESS)
  {
    LOG_ERROR("Failed to initialize wifi in Clarius OEM device");
    this->InternalDisconnect();
    return PLUS_FAIL;
  }

  // OEM library
  if (this->InitializeOEM() != PLUS_SUCCESS)
  {
    LOG_ERROR("Failed to initialize OEM library in Clarius OEM device");
    this->InternalDisconnect();
    return PLUS_FAIL;
  }

  // SET CERTIFICATE
  if (this->SetClariusCert() != PLUS_SUCCESS)
  {
    LOG_ERROR("Failed to set Clarius certificate. Please check your PathToCert is valid, and contains the correct cert for the probe you're connecting to");
    this->InternalDisconnect();
    return PLUS_FAIL;
  }

  // CONFIGURE PROBE MODE
  if (this->ConfigureProbeApplication() != PLUS_SUCCESS)
  {
    LOG_ERROR("Failed to configure Clarius probe application");
    this->InternalDisconnect();
    return PLUS_FAIL;
  }

  // at this point the probe is connected, we set this->Connected here
  // to allow the call to this->SetInitialUsParams() at the end of 
  // internal connect to actually set the params (rather than caching them)
  this->Connected = true;

  // print device stats
  ClariusStatusInfo stats;
  if (cusOemStatusInfo(&stats) == 0)
  {
    LOG_INFO("battery: " << stats.battery << "%, temperature: " << stats.temperature << "%");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_LONG_DELAY_MS));

  // enable the 5v rail on the top of the Clarius probe
  int enable5v = this->Internal->Enable5vRail ? 1 : 0;
  if (cusOemEnable5v(enable5v) < 0)
  {
    std::string enstr = (enable5v ? "TRUE" : "FALSE");
    LOG_WARNING("Failed to set the state of the Clarius probe 5v rail, provided enable value was: " << enstr);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_LONG_DELAY_MS));

  // set imaging parameters
  this->SetInitialUsParams();

  return PLUS_SUCCESS;
};

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::DeInitializeOEM()
{
  int oemState = cusOemIsConnected();

  if (oemState == CLARIUS_STATE_CONNECTED)
  {
    if (cusOemEnable5v(CLARIUS_FALSE) < 0)
    {
      LOG_WARNING("Failed to disable Clarius 5v");
    }
    if (cusOemRun(CLARIUS_STOP) < 0)
    {
      LOG_WARNING("Failed to stop Clarius imaging");
    }
    if (cusOemDisconnect() < 0)
    {
      LOG_WARNING("Failed to disconnect from Clarius OEM library");
    }
    if (cusOemDestroy() < 0)
    {
      LOG_WARNING("Failed to destroy Clarius OEM library");
    }
  }
  else if (oemState == CLARIUS_STATE_NOT_CONNECTED)
  {
    if (cusOemDestroy() < 0)
    {
      LOG_WARNING("Failed to destroy Clarius OEM library");
    }
  }
  else if (oemState == CLARIUS_STATE_NOT_INITIALIZED)
  {
    // nothing to do here
  }
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::DeInitializeWifi()
{
  if (!this->Internal->WifiHelper.DisconnectFromClariusWifi())
  {
    LOG_WARNING("Failed to disconnect from Clarius wifi");
  }
  if (!this->Internal->WifiHelper.DeInitialize())
  {
    LOG_WARNING("Failed to de-initialize ClariusWifiHelper");
  }
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::DeInitializeProbe()
{
  // power off the probe
  // TODO: fix the exception this causes
  // bool powerState;
  // if (!this->Internal->BluetoothInterface->PowerProbe(false, powerState))
  // {
  //   LOG_WARNING("An error occurred when powering down Clarius probe, error text was: "
  //     << this->Internal->BluetoothInterface->GetLastErrorMessage());
  // }

  // if (powerState)
  // {
  //   LOG_WARNING("Failed to power off Clarius probe");
  // }
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::DeInitializeBLE()
{
  // disconnect from probe
  // TODO: fix the exception this causes
  // bool disconnectionResult;
  // if (!this->Internal->BluetoothInterface->DisconnectFromProbeBT(disconnectionResult))
  // {
  //   LOG_WARNING("An error occurred during DisconnectFromProbeBT, error text was: "
  //     << this->Internal->BluetoothInterface->GetLastErrorMessage());
  // }

  // if (!disconnectionResult)
  // {
  //   LOG_WARNING("Failed to disconnect from Clarius probe");
  // }

  // // de-initialize interface
  // if (!this->Internal->BluetoothInterface->DeInitialize())
  // {
  //   LOG_WARNING("Failed to de-initialize BluetoothInterface");
  // }

  // if (this->Internal->BluetoothInterface)
  // {
  //   delete this->Internal->BluetoothInterface;
  //   this->Internal->BluetoothInterface = nullptr;
  // }
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalDisconnect()
{
  LOG_TRACE("vtkPlusClariusOEM::InternalDisconnect");

  // inverse order to initialization
  this->DeInitializeOEM();
  this->DeInitializeWifi();
  this->DeInitializeProbe();
  this->DeInitializeBLE();

  return PLUS_SUCCESS;
};

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalStartRecording()
{
  LOG_TRACE("vtkPlusClariusOEM::InternalStartRecording");

  if (cusOemRun(CLARIUS_RUN) < 0)
  {
    LOG_ERROR("Failed to start Clarius imaging");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_LONG_DELAY_MS));

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalStopRecording()
{
  LOG_TRACE("vtkPlusClariusOEM::InternalStopRecording");

  if (cusOemRun(CLARIUS_STOP) < 0)
  {
    LOG_ERROR("Failed to stop Clarius imaging");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_LONG_DELAY_MS));

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalApplyImagingParameterChange()
{
  PlusStatus status = PLUS_SUCCESS;

  // depth (mm), note: Clarius uses cm
  if (this->ImagingParameters->IsSet(vtkPlusUsImagingParameters::KEY_DEPTH)
    && this->ImagingParameters->IsPending(vtkPlusUsImagingParameters::KEY_DEPTH))
  {
    if (this->SetDepthMm(this->ImagingParameters->GetDepthMm()) != PLUS_SUCCESS)
    {
      LOG_ERROR("Failed to set depth imaging parameter");
      status = PLUS_FAIL;
    }
    this->ImagingParameters->SetPending(vtkPlusUsImagingParameters::KEY_DEPTH, false);
  }

  // gain (percent)
  if (this->ImagingParameters->IsSet(vtkPlusUsImagingParameters::KEY_GAIN)
    && this->ImagingParameters->IsPending(vtkPlusUsImagingParameters::KEY_GAIN))
  {
    if (this->SetGainPercent(this->ImagingParameters->GetGainPercent()) != PLUS_SUCCESS)
    {
      LOG_ERROR("Failed to set gain imaging parameter");
      status = PLUS_FAIL;
    }
    this->ImagingParameters->SetPending(vtkPlusUsImagingParameters::KEY_GAIN, false);
  }

  // dynamic range (percent)
  if (this->ImagingParameters->IsSet(vtkPlusUsImagingParameters::KEY_DYNRANGE)
    && this->ImagingParameters->IsPending(vtkPlusUsImagingParameters::KEY_DYNRANGE))
  {
    if (this->SetDynRangePercent(this->ImagingParameters->GetDynRangeDb()) != PLUS_SUCCESS)
    {
      LOG_ERROR("Failed to set dynamic range imaging parameter");
      status = PLUS_FAIL;
    }
    this->ImagingParameters->SetPending(vtkPlusUsImagingParameters::KEY_DYNRANGE, false);
  }

  // TGC (time gain compensation)
  if (this->ImagingParameters->IsSet(vtkPlusUsImagingParameters::KEY_TGC)
    && this->ImagingParameters->IsPending(vtkPlusUsImagingParameters::KEY_TGC))
  {
    std::vector<double> tgcVec;
    this->ImagingParameters->GetTimeGainCompensation(tgcVec);
    if (this->SetTimeGainCompensationPercent(tgcVec) != PLUS_SUCCESS)
    {
      LOG_ERROR("Failed to set time gain compensation imaging parameter");
      status = PLUS_FAIL;
    }
    this->ImagingParameters->SetPending(vtkPlusUsImagingParameters::KEY_TGC, false);
  }

  return status;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::GetDepthMm(double& aDepthMm)
{
  if (!this->Connected)
  {
    // Connection has not been established yet, return cached parameter value
    return this->ImagingParameters->GetDepthMm(aDepthMm);
  }

  double oemVal = cusOemGetParam(PARAM_DEPTH);
  if (oemVal < 0)
  {
    aDepthMm = -1;
    LOG_ERROR("Failed to get DepthMm parameter");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_SHORT_DELAY_MS));

  aDepthMm = oemVal * CM_TO_MM;

  // ensure ImagingParameters is up to date
  this->ImagingParameters->SetDepthMm(aDepthMm);

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::SetDepthMm(double aDepthMm)
{
  if (!this->Connected)
  {
    // Connection has not been established yet, parameter value will be set upon connection
    this->ImagingParameters->SetDepthMm(aDepthMm);
    LOG_INFO("Cached US parameter DepthMm = " << aDepthMm);
    return PLUS_SUCCESS;
  }

  // attempt to set parameter value
  double depthCm = aDepthMm * MM_TO_CM;
  if (cusOemSetParam(PARAM_DEPTH, depthCm) < 0)
  {
    LOG_ERROR("Failed to set DepthMm parameter");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_SHORT_DELAY_MS));

  // update imaging parameters & return successfully
  this->ImagingParameters->SetDepthMm(aDepthMm);
  LOG_INFO("Set US parameter DepthMm to " << aDepthMm);
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::GetGainPercent(double& aGainPercent)
{
  if (!this->Connected)
  {
    // Connection has not been established yet, return cached parameter value
    return this->ImagingParameters->GetGainPercent(aGainPercent);
  }

  double oemVal = cusOemGetParam(PARAM_GAIN);
  if (oemVal < 0)
  {
    aGainPercent = -1;
    LOG_ERROR("Failed to get GainPercent parameter");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_SHORT_DELAY_MS));

  aGainPercent = oemVal;

  // ensure ImagingParameters is up to date
  this->ImagingParameters->SetGainPercent(aGainPercent);

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::SetGainPercent(double aGainPercent)
{
  if (!this->Connected)
  {
    // Connection has not been established yet, parameter value will be set upon connection
    this->ImagingParameters->SetGainPercent(aGainPercent);
    LOG_INFO("Cached US parameter GainPercent = " << aGainPercent);
    return PLUS_SUCCESS;
  }

  // attempt to set parameter value
  if (cusOemSetParam(PARAM_GAIN, aGainPercent) < 0)
  {
    LOG_ERROR("Failed to set GainPercent parameter");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_SHORT_DELAY_MS));

  // update imaging parameters & return successfully
  this->ImagingParameters->SetGainPercent(aGainPercent);
  LOG_INFO("Set US parameter GainPercent to " << aGainPercent);
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::GetDynRangePercent(double& aDynRangePercent)
{
  if (!this->Connected)
  {
    // Connection has not been established yet, return cached parameter value
    return this->ImagingParameters->GetDynRangeDb(aDynRangePercent);
  }

  double oemVal = cusOemGetParam(PARAM_DYNRNG);
  if (oemVal < 0)
  {
    aDynRangePercent = -1;
    LOG_ERROR("Failed to get DynRange parameter");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_SHORT_DELAY_MS));

  aDynRangePercent = oemVal;

  // ensure ImagingParameters is up to date
  this->ImagingParameters->SetDynRangeDb(aDynRangePercent);

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::SetDynRangePercent(double aDynRangePercent)
{
  if (!this->Connected)
  {
    // Connection has not been established yet, parameter value will be set upon connection
    this->ImagingParameters->SetDynRangeDb(aDynRangePercent);
    LOG_INFO("Cached US parameter DynRangePercent = " << aDynRangePercent);
    return PLUS_SUCCESS;
  }

  // attempt to set parameter value
  if (cusOemSetParam(PARAM_DYNRNG, aDynRangePercent) < 0)
  {
    LOG_ERROR("Failed to set DynRange parameter");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_SHORT_DELAY_MS));

  // update imaging parameters & return successfully
  this->ImagingParameters->SetDynRangeDb(aDynRangePercent);
  LOG_INFO("Set US parameter DynRangePercent to " << aDynRangePercent);
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::GetTimeGainCompensationPercent(std::vector<double>& aTGC)
{
  if (!this->Connected)
  {
    // Connection has not been established yet, return cached parameter value
    return this->ImagingParameters->GetTimeGainCompensation(aTGC);
  }

  ClariusTgc cTGC;
  if (cusOemGetTgc(&cTGC) < 0)
  {
    LOG_ERROR("Failed to get time gain compensation parameter");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_SHORT_DELAY_MS));

  aTGC.clear();
  aTGC.resize(3);
  aTGC[0] = cTGC.top;
  aTGC[1] = cTGC.mid;
  aTGC[2] = cTGC.bottom;

  // ensure imaging parameters are up to date
  this->ImagingParameters->SetTimeGainCompensation(aTGC);

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::SetTimeGainCompensationPercent(const std::vector<double>& aTGC)
{
  if (aTGC.size() != 3)
  {
    LOG_ERROR("vtkPlusClariusOEM time gain compensation parameter must be provided a vector of exactly 3 doubles [top gain, mid gain, bottom gain]");
    return PLUS_FAIL;
  }

  if (!this->Connected)
  {
    // Connection has not been established yet, parameter value will be set upon connection
    this->ImagingParameters->SetTimeGainCompensation(aTGC);
    LOG_INFO("Cached US parameter TGC = [" << aTGC[0] << ", " << aTGC[1] << ", " << aTGC[2] << "]");
    return PLUS_SUCCESS;
  }

  ClariusTgc cTGC;
  cTGC.top = aTGC[0];
  cTGC.mid = aTGC[1];
  cTGC.bottom = aTGC[2];
  if (cusOemSetTgc(&cTGC) < 0)
  {
    LOG_ERROR("Failed to set time gain compensation parameter");
    return PLUS_FAIL;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(CLARIUS_SHORT_DELAY_MS));

  // update imaging parameters & return successfully
  this->ImagingParameters->SetTimeGainCompensation(aTGC);
  LOG_INFO("Set US parameter TGC to [" << aTGC[0] << ", " << aTGC[1] << ", " << aTGC[2] << "]");
  return PLUS_SUCCESS;
}
