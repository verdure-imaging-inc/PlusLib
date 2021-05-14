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

// IGSIO includes
#include <vtkIGSIOAccurateTimer.h>

// VTK includes
#include <vtkImageData.h>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// STL includes
#include <chrono>
#include <map>
#include <sstream>
#include <string>

// Clarius bluetooth helper includes
#include <CBTInterface.h>

// Clarius Includes
#include <oem.h>
#include <oem_def.h>


namespace
{
  // TODO: convert this to FrameSizeType
  static const int DEFAULT_FRAME_WIDTH = 640;
  static const int DEFAULT_FRAME_HEIGHT = 480;

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

  // Clarius Qt bluetooth library
  CBTInterface* BluetoothInterface;
  
  // user configurable params
  std::string ProbeSerialNum;
  std::string PathToCert;

  // system parameters
  std::string IpAddress;
  int TcpPort;

  // is there currently a probe connected
  // TODO: rename this OemConnected or OemProbeConnected
  bool ProbeConnected;

private:
  vtkPlusClariusOEM* External;
};

//-------------------------------------------------------------------------------------------------
vtkPlusClariusOEM::vtkInternal::vtkInternal(vtkPlusClariusOEM* ext)
: External(ext)
, BluetoothInterface(nullptr)
, ProbeSerialNum("")
, PathToCert("")
, IpAddress("")
, TcpPort(-1)
, ProbeConnected(false)
{
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::ListFn(const char* list, int sz)
{
  LOG_INFO("ListFn: " << list);
}

//-------------------------------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::ConnectFn(int ret, int port, const char* status)
{
  if (ret == CONNECT_SUCCESS)
  {
    // connection succeeded, set Internal->Connected variable to end busy wait in InternalConnect
    vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
    device->Internal->ProbeConnected = true;
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
  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
  if (device == NULL)
  {
    LOG_ERROR("Clarius instance is NULL!!!");
    return;
  }


  LOG_TRACE("new image (" << newImage << "): " << nfo->width << " x " << nfo->height << " @ " << nfo->bitsPerPixel
    << "bits. @ " << nfo->micronsPerPixel << " microns per pixel. imu points: " << npos);

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
  LOG_INFO("error: " << err);
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
  this->ImagingParameters->SetImageSize(DEFAULT_FRAME_WIDTH, DEFAULT_FRAME_HEIGHT, 1);

  this->RequirePortNameInDeviceSetConfiguration = true;

  instance = this;
}

//-------------------------------------------------------------------------------------------------
vtkPlusClariusOEM::~vtkPlusClariusOEM()
{
  if (this->Recording)
  {
    this->StopRecording();
  }

  if (this->Connected)
  {
    cusOemDisconnect();
  }

  int destroyed = cusOemDestroy();
  if (destroyed != 0)
  {
    LOG_ERROR("Error destoying the listener");
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
  FrameSizeType fs = this->ImagingParameters->GetImageSize();
  os << indent << "FrameWidth" << fs[0] << std::endl;
  os << indent << "FrameHeight" << fs[1] << std::endl;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::ParseConnectionConfig(vtkXMLDataElement* deviceConfig)
{
  LOG_TRACE("vtkPlusClariusOEM::ParseConnectionConfig");


  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::ParseGainConfig(vtkXMLDataElement* deviceConfig)
{
  LOG_TRACE("vtkPlusClariusOEM::ParseGainConfig");

  return PLUS_SUCCESS;
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

  // TODO: frame size
  

  // probe serial number
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(
    ProbeSerialNum, this->Internal->ProbeSerialNum, deviceConfig);

  // path to Clarius certificate
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(
    PathToCert, this->Internal->PathToCert, deviceConfig);
  
  // TODO: print configuration
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusClariusOEM::WriteConfiguration");

  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);

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
PlusStatus vtkPlusClariusOEM::PowerOnClarius(vtkPlusClariusOEM* device)
{
  if (!this->Internal->BluetoothInterface)
  {
    this->Internal->BluetoothInterface = new CBTInterface();
    this->Internal->BluetoothInterface->Initialize();
  }

  // search for probes
  std::vector<std::string> probes;
  if (!this->Internal->BluetoothInterface->SearchForProbes(probes))
  {
    LOG_ERROR("An error occurred during SearchForProbes. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  if (probes.size() == 0)
  {
    LOG_ERROR("No Clarius probes found... Please check probe has battery, and is connected to Windows bluetooth");
    return PLUS_FAIL;
  }

  // is desired probe available
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

  bool connectionResult = false;
  int tries = 0;
  while (tries < 100)
  {
    if (!this->Internal->BluetoothInterface->ConnectToProbeBT(this->Internal->ProbeSerialNum, connectionResult))
    {
      // LOG_ERROR("An error occurred during ConnectToProbeBT. Error text: "
      //  << this->Internal->BluetoothInterface->GetLastErrorMessage());
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


  // TODO: check the probe is connected
  bool isConnected;
  if (!this->Internal->BluetoothInterface->IsProbeConnected(isConnected))
  {
    LOG_ERROR("An error occurred during ConnectToProbeBT. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  if (!isConnected)
  {
    LOG_ERROR("Somehow checking IsProbeConnected returns FALSE, when probe connection succeeded");
    return PLUS_FAIL;
  }

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
  std::this_thread::sleep_for(std::chrono::milliseconds(3500));

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
    LOG_ERROR("Wifi was not connceted when wifi info was requested.");
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

    this->Internal->IpAddress = info.IPv4;
    this->Internal->TcpPort = info.ControlPort;
  }

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InitializeClarius(vtkPlusClariusOEM* device)
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
  device->GetVideoSourcesByPortName(vtkPlusDevice::BMODE_PORT_NAME, bModeSources);
  if (bModeSources.empty())
  {
    newProcessedImageFnPtr = nullptr;
  }

  // no RF-mode data sources, disable RF-mode callback
  std::vector<vtkPlusDataSource*> rfModeSources;
  device->GetVideoSourcesByPortName(vtkPlusDevice::RFMODE_PORT_NAME, rfModeSources);
  if (rfModeSources.empty())
  {
    newRawImageFnPtr = nullptr;
  }

  try
  {
    FrameSizeType fs = this->ImagingParameters->GetImageSize();

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

    if (result != 0)
    {
      LOG_ERROR("Failed to initialize Clarius OEM library")
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
PlusStatus vtkPlusClariusOEM::SetClariusCert(vtkPlusClariusOEM* device)
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

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::ConnectToClarius(vtkPlusClariusOEM* device)
{
  const char* ip = device->Internal->IpAddress.c_str();
  unsigned int port = device->Internal->TcpPort;
  LOG_INFO("Attempting connection to Clarius ultrasound on " << ip << ":" << port << " for 10 seconds:");

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

  // get start timestamp
  std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();

  while (!Internal->ProbeConnected)
  {
    std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
    std::chrono::duration<double> dur = t - startTime;
    if (dur.count() > 10)
    {
      LOG_ERROR("Connection to Clarius device timed out.");
      return PLUS_FAIL;
    }
  }

  LOG_INFO("Connected to Clarius probe on " << ip << ":" << port);
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalConnect()
{
  LOG_TRACE("vtkPlusClariusOEM::InternalConnect");

  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();

  if (!this->Internal->ProbeConnected)
  {
    // power on the probe & get TCP/IP values
    if (this->PowerOnClarius(device) != PLUS_SUCCESS)
    {
      LOG_ERROR("Failed to power on Clarius probe");
      return PLUS_FAIL;
    }
    
    // initialize Clarius OEM library
    if (this->InitializeClarius(device) != PLUS_SUCCESS)
    {
      cusOemDestroy();
      this->PowerOffClarius(device);
      LOG_ERROR("Failed to initalize Clarius.");
      return PLUS_FAIL;
    }

    // set Clarius certificate
    if (this->SetClariusCert(device) != PLUS_SUCCESS)
    {
      this->PowerOffClarius(device);
      cusOemDestroy();
      LOG_ERROR("Failed to set Clarius certificate. Please check your PathToCert is valid, and contains the correct cert for the probe you're connecting to.");
      return PLUS_FAIL;
    }

    // connect to Clarius probe
    if (this->ConnectToClarius(device) != PLUS_SUCCESS)
    {
      this->PowerOffClarius(device);
      cusOemDestroy();
      LOG_ERROR("Failed to connect to Clarius probe.");
      return PLUS_FAIL;
    }
  }
  else
  {
    LOG_ERROR("Scanner already connected");
  }

  vtkIGSIOAccurateTimer::Delay(1.0);

  ClariusStatusInfo stats;
  if (cusOemStatusInfo(&stats) == 0)
    LOG_INFO("battery: " << stats.battery << "%, temperature: " << stats.temperature << "%");


  // configure probe
  if (cusOemLoadApplication("C3HD", "msk") == 0)
  {
    LOG_INFO("trying to load application: probe=" << "C3HD" << " application=" << "msk");
  }
  else
  {
    LOG_ERROR("error calling load application");
  }

  vtkIGSIOAccurateTimer::Delay(1.0);

  return PLUS_SUCCESS;
};

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::PowerOffClarius(vtkPlusClariusOEM* device)
{
  // power off the probe
  bool powerState;
  if (!this->Internal->BluetoothInterface->PowerProbe(false, powerState))
  {
    LOG_ERROR("An error occurred during PowerProbe. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  if (powerState)
  {
    LOG_ERROR("Failed to power off Clarius probe");
    return PLUS_FAIL;
  }

  // disconnect from probe
  bool disconnectionResult;
  if (!this->Internal->BluetoothInterface->DisconnectFromProbeBT(disconnectionResult))
  {
    LOG_ERROR("An error occurred during DisconnectFromProbeBT. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  if (!disconnectionResult)
  {
    LOG_ERROR("Failed to disconnect from Clarius probe");
    return PLUS_FAIL;
  }

  // check if disconnected
  bool isConnected;
  if (!this->Internal->BluetoothInterface->IsProbeConnected(isConnected))
  {
    LOG_ERROR("An error occurred during IsProbeConnected. Error text: "
      << this->Internal->BluetoothInterface->GetLastErrorMessage());
    return PLUS_FAIL;
  }

  if (!isConnected)
  {
    LOG_ERROR("Somehow checking IsProbeConnected returns FALSE, when probe disconnection succeeded");
    return PLUS_FAIL;
  }

  // de-initialize interface
  this->Internal->BluetoothInterface->DeInitialize();
  delete this->Internal->BluetoothInterface;
  this->Internal->BluetoothInterface = nullptr;

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalDisconnect()
{
  LOG_TRACE("vtkPlusClariusOEM::InternalDisconnect");

  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
  if (this->Internal->ProbeConnected)
  {
    if (cusOemDisconnect() < 0)
    {
      LOG_ERROR("could not disconnect from scanner");
      return PLUS_FAIL;
    }
    else
    {
      device->Connected = 0;
      LOG_DEBUG("Clarius device is now disconnected");
      return PLUS_SUCCESS;
    }

    if (this->PowerOffClarius(device) != PLUS_SUCCESS)
    {
      LOG_ERROR("Failed to power off Clarius probe");
      return PLUS_FAIL;
    }
  }
  else
  {
    LOG_DEBUG("...Clarius device already disconnected");
    return PLUS_SUCCESS;
  }
};

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalStartRecording()
{
  LOG_TRACE("vtkPlusClariusOEM::InternalStartRecording");

  if (cusOemRun(1) < 0)
  {
    LOG_ERROR("run imaging request failed");
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalStopRecording()
{
  LOG_TRACE("vtkPlusClariusOEM::InternalStopRecording");

  if (cusOemRun(0) < 0)
  {
    LOG_ERROR("stop imaging request failed");
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;
}
