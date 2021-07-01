/*=Plus=header=begin======================================================
    Program: Plus
    Copyright (c) Verdure Imaging Inc, Stockton, California. All rights reserved.
    See License.txt for details.

    We would like to acknowledge Verdure Imaging Inc for generously open-sourcing
    this support for the Clarius OEM interface to the PLUS & Slicer communities.
=========================================================Plus=header=end*/

#ifndef _VTKPLUSCLARIUSOEM_H
#define _VTKPLUSCLARIUSOEM_H

// Local includes
#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusUsDevice.h"

// STL includes
#include <vector>

/*!
\class vtkPlusClariusOEM
\brief Interface to Clarius Ultrasound Devices
This class talks with a Clarius US Scanner over the Clarius OEM API.
Requires the PLUS_USE_CLARIUS_OEM option in CMake.
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

public:

  PlusStatus ParseImuConfig(vtkXMLDataElement* deviceConfig);

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

  /*! return the singleton instance with no reference counting */
  static vtkPlusClariusOEM* GetInstance();

protected:
  vtkPlusClariusOEM();
  ~vtkPlusClariusOEM();

protected:

  PlusStatus InitializeBLE();

  PlusStatus InitializeProbe();

  PlusStatus InitializeWifi();

  PlusStatus InitializeOEM();

  PlusStatus SetClariusCert();

  PlusStatus ConfigureProbeApplication();

  PlusStatus SetInitialUsParams();

  PlusStatus InternalConnect() override;

  void DeInitializeOEM();

  void DeInitializeWifi();

  void DeInitializeProbe();

  void DeInitializeBLE();

  PlusStatus InternalDisconnect() override;

  PlusStatus InternalStartRecording() override;

  PlusStatus InternalStopRecording() override;

  PlusStatus InternalApplyImagingParameterChange() override;

public:

  // US parameters API

  /*! Get the imaging depth of B-mode ultrasound (mm) */
  PlusStatus GetDepthMm(double& aDepthMm);
  /*! Set the imaging depth of B-mode ultrasound (mm) */
  PlusStatus SetDepthMm(double aDepthMm);

  /*! Get the gain percentage of B-mode ultrasound (%) */
  PlusStatus GetGainPercent(double& aGainPercent);
  /*! Set the gain percentage of B-mode ultrasound (%) */
  PlusStatus SetGainPercent(double aGainPercent);

  /*! Get the dynamic range of B-mode ultrasound (%)*/
  PlusStatus GetDynRangePercent(double& aDynamicRangePercent);
  /*! Set the dynamic range of B-mode ultrasound (%)*/
  PlusStatus SetDynRangePercent(double aDynamicRangePercent);

  /*! Get the time gain compensation in (%) */
  PlusStatus GetTimeGainCompensationPercent(std::vector<double>& aTGC);
  /*! Set the time gain compensation (%) */
  PlusStatus SetTimeGainCompensationPercent(const std::vector<double>& aTGC);

private:

  static vtkPlusClariusOEM* instance;

  class vtkInternal;
  vtkInternal* Internal;
};

#endif
