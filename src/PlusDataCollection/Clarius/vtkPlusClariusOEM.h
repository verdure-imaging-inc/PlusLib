/*=Plus=header=begin======================================================
    Program: Plus
    Copyright (c) UBC Biomedical Signal and Image Computing Laboratory. All rights reserved.
=========================================================Plus=header=end*/

#ifndef _VTKPLUSCLARIUSOEM_H
#define _VTKPLUSCLARIUSOEM_H

// Local includes
#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusUsDevice.h"

// Clarius Includes
#include "oem.h"
#include "oem_def.h"

// OpenCV includes
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

class AhrsAlgo;

/*!
\class vtkPlusClariusOEM
\brief Interface to the Clarius OEM API
This class talks with a Clarius Scanner over the Clarius OEM API.
Note: A license may be required from Clarius to access this API.
Requires PLUS_USE_CLARIUS_OEM option in CMake.
 \ingroup PlusLibDataCollection
*/
class vtkPlusDataCollectionExport vtkPlusClariusOEM : public vtkPlusUsDevice
{
public:
  vtkTypeMacro(vtkPlusClariusOEM, vtkPlusUsDevice);
  static vtkPlusClariusOEM* New();
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /*! Probe to see to see if the device is connected to the
  computer. This method should be overridden in subclasses. */
  PlusStatus Probe() override;

  /*! Hardware device SDK version. This method should be overridden in subclasses. */
  std::string GetSdkVersion() override;

  /*! The IMU streaming is supported and raw IMU data is written to csv file, however interpreting imu data as tracking data is not supported*/
  bool IsTracker() const override { return false; };

  bool IsVirtual() const override { return false; };

  /*! Read configuration from xml data */
  PlusStatus ReadConfiguration(vtkXMLDataElement* config) override;

  /*! Write configuration to xml data */
  PlusStatus WriteConfiguration(vtkXMLDataElement* config) override;

  /*! Perform any completion tasks once configured
   a multi-purpose function which is called after all devices have been configured,
   all inputs and outputs have been connected between devices,
   but before devices begin collecting data.
   This is the last chance for your device to raise an error about improper or insufficient configuration.
  */
  PlusStatus NotifyConfigured() override;

protected:
  PlusStatus InternalConnect() override;

  PlusStatus InternalDisconnect() override;

  PlusStatus InternalStartRecording() override;

  PlusStatus InternalStopRecording() override;

public:

  /*! return the singleton instance with no reference counting */
  static vtkPlusClariusOEM* GetInstance();

  /*!
  Request raw ultrasound data in the last N seconds
  */
  PlusStatus RequestLastNSecondsRawData(double lastNSeconds);

  /*!
  Request raw ultrasound data between two timestamps
  If both timestamps are zero, all available data will be requested
  */
  PlusStatus RequestRawData(long long startTimestampNanoSeconds, long long endTimestampNanoSeconds);

  vtkSetMacro(FrameHeight, unsigned int);
  vtkGetMacro(FrameHeight, unsigned int);

  vtkSetMacro(FrameWidth, unsigned int);
  vtkGetMacro(FrameWidth, unsigned int);

  vtkSetMacro(IpAddress, std::string);
  vtkGetMacro(IpAddress, std::string);

  vtkSetMacro(TcpPort, unsigned int);
  vtkGetMacro(TcpPort, unsigned int);

  vtkSetMacro(ImuEnabled, bool);
  vtkGetMacro(ImuEnabled, bool);

  vtkSetMacro(WriteImagesToDisk, bool);
  vtkGetMacro(WriteImagesToDisk, bool);

  vtkSetMacro(ImuOutputFileName, std::string);
  vtkGetMacro(ImuOutputFileName, std::string);

  /*!
  Compress raw data using gzip if enabled
  */
  vtkGetMacro(CompressRawData, bool);
  vtkSetMacro(CompressRawData, bool);
  vtkBooleanMacro(CompressRawData, bool);

  /*!
  Output filename of the raw Clarius data
  If empty, data will be written to the Plus output directory
  */
  vtkSetStdStringMacro(RawDataOutputFilename);
  vtkGetStdStringMacro(RawDataOutputFilename);

protected:
  vtkPlusClariusOEM();
  ~vtkPlusClariusOEM();


  PlusStatus WritePosesToCsv(const ClariusProcessedImageInfo* nfo, int npos, const ClariusPosInfo* pos, int frameNum, double systemTime, double convertedTime);

  /*!
  Receive previously requested data
  */
  PlusStatus ReceiveRawData(int dataSize);

protected:
  unsigned int TcpPort;
  int UdpPort;
  std::string IpAddress;
  std::string PathToSecKey; // path to security key, required by the clarius api
  std::ofstream RawImuDataStream;
  std::string ImuOutputFileName;
  int FrameWidth;
  int FrameHeight;
  int FrameNumber;
  double SystemStartTimestamp;
  double ClariusStartTimestamp;
  double ClariusLastTimestamp;
  bool ImuEnabled;
  bool WriteImagesToDisk;

  cv::Mat cvImage;

  bool CompressRawData;
  bool IsReceivingRawData;
  int RawDataSize;
  void* RawDataPointer;
  std::string RawDataOutputFilename;

  static vtkPlusClariusOEM* instance;



  /*!
  Callback function for raw data request
  */
  static void RawDataRequestFn(int rawDataSize);

  /*!
  Callback function for raw data read
  */
  static void RawDataWriteFn(int rawDataSize);

  /*!
  Re-allocate memory to store raw ultrasound data
  */
  void AllocateRawData(int size);

 // TODO: move to PIMPL class
  vtkPlusDataSource* AccelerometerTool;
  vtkPlusDataSource* GyroscopeTool;
  vtkPlusDataSource* MagnetometerTool;
  vtkPlusDataSource* TiltSensorTool;
  vtkPlusDataSource* FilteredTiltSensorTool;
  vtkPlusDataSource* OrientationSensorTool;
  vtkNew<vtkMatrix4x4> LastAccelerometerToTrackerTransform;
  vtkNew<vtkMatrix4x4> LastGyroscopeToTrackerTransform;
  vtkNew<vtkMatrix4x4> LastMagnetometerToTrackerTransform;
  vtkNew<vtkMatrix4x4> LastTiltSensorToTrackerTransform;
  vtkNew<vtkMatrix4x4> LastFilteredTiltSensorToTrackerTransform;
  vtkNew<vtkMatrix4x4> LastOrientationSensorToTrackerTransform;

  enum AHRS_METHOD
  {
    AHRS_MADGWICK,
    AHRS_MAHONY
  };

  AhrsAlgo* FilteredTiltSensorAhrsAlgo;

  AhrsAlgo* AhrsAlgo;

  /*!
    If AhrsUseMagnetometer enabled (a ..._MARG algorithm is chosen) then heading will be estimated using magnetometer data.
    Otherwise (when a ..._IMU algorithm is chosen) only the gyroscope data will be used for getting the heading information.
    IMU may be more noisy, but not sensitive to magnetic field distortions.
  */
  bool AhrsUseMagnetometer;

  /*!
    Gain values used by the AHRS algorithm (Mahony: first parameter is proportional, second is integral gain; Madgwick: only the first parameter is used)
    Higher gain gives higher reliability to accelerometer&magnetometer data.
  */
  double AhrsAlgorithmGain[2];
  double FilteredTiltSensorAhrsAlgorithmGain[2];
  vtkSetVector2Macro(AhrsAlgorithmGain, double);
  vtkSetVector2Macro(FilteredTiltSensorAhrsAlgorithmGain, double);

  /*! last AHRS update time (in system time) */
  double AhrsLastUpdateTime;
  double FilteredTiltSensorAhrsLastUpdateTime;

  /*!
    In tilt sensor mode we don't use the magnetometer, so we have to provide a direction reference.
    The orientation is specified by specifying an axis that will always point to the "West" direction.
    Recommended values:
    If sensor axis 0 points down (the sensor plane is about vertical) => TiltSensorDownAxisIndex = 2.
    If sensor axis 1 points down (the sensor plane is about vertical) => TiltSensorDownAxisIndex = 0.
    If sensor axis 2 points down (the sensor plane is about horizontal) => TiltSensorDownAxisIndex = 1.
  */
  int TiltSensorWestAxisIndex;
  int FilteredTiltSensorWestAxisIndex;

  class vtkInternal;
  vtkInternal* Internal;
};

#endif
