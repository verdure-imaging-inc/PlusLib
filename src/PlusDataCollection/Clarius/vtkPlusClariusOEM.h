/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef _VTKPLUSCLARIUSOEM_H
#define _VTKPLUSCLARIUSOEM_H

// Local includes
#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusUsDevice.h"

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

private:

  // configuration helper methods

  PlusStatus ParseConnectionConfig(vtkXMLDataElement* deviceConfig);

  PlusStatus ParseGainConfig(vtkXMLDataElement* deviceConfig);

  PlusStatus ParseImuConfig(vtkXMLDataElement* deviceConfig);

public:
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

  PlusStatus InternalConnect() override;

  PlusStatus InternalDisconnect() override;

  PlusStatus InternalStartRecording() override;

  PlusStatus InternalStopRecording() override;

  static vtkPlusClariusOEM* instance;

  class vtkInternal;
  vtkInternal* Internal;

};

#endif
