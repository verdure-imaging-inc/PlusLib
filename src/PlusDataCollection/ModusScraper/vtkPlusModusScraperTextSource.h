/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusModusScraperTextSource_h_
#define __vtkPlusModusScraperTextSource_h_

#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusDevice.h"

/*!
\class vtkPlusModusScraperTextSource
\brief Class for scraping the Modus developer website for needed variables

\ingroup PlusLibDataCollection
*/

class vtkPlusDataCollectionExport vtkPlusModusScraperTextSource : public vtkPlusDevice
{
public:
  static vtkPlusModusScraperTextSource* New();
  vtkTypeMacro(vtkPlusModusScraperTextSource, vtkPlusDevice);
  virtual void PrintSelf(ostream& os, vtkIndent indent) VTK_OVERRIDE;

  /*! Read configuration from xml data */
  virtual PlusStatus ReadConfiguration(vtkXMLDataElement* config);
  /*! Write configuration to xml data */
  virtual PlusStatus WriteConfiguration(vtkXMLDataElement* config);

  /*! Manage device frozen state */
  PlusStatus FreezeDevice(bool freeze);
  virtual bool IsTracker() const { return false; }
  virtual PlusStatus InternalUpdate();
  virtual PlusStatus NotifyConfigured();

  vtkGetStdStringMacro(URL);
  vtkSetStdStringMacro(URL);

protected:
  vtkPlusModusScraperTextSource();
  ~vtkPlusModusScraperTextSource();

  virtual PlusStatus InternalConnect();
  virtual PlusStatus InternalDisconnect();

protected:
  std::string                       URL;
};

#endif // __vtkPlusModusScraperTextSource_h_
