/*=Plus=header=begin======================================================
    Program: Plus
    Copyright (c) Verdure Imaging Inc, Stockton, California. All rights reserved.
    See License.txt for details.

    Dynamic loader for Clarius Solum SDK - enables a single PLUS binary
    to work with any compatible version of solum.dll at runtime.

    USAGE:
    Replace #include <solum.h> with #include "SolumDynamicLoader.h"
    Call SolumDynLoader::Load() before any solum calls (in InitializeOEM)
    Call SolumDynLoader::Unload() on shutdown (in DeInitializeOEM)
    All solum* function calls work unchanged via macro redirection.

    The loader searches for solum.dll in:
      1. Path specified by SOLUM_DLL_PATH environment variable
      2. Application directory
      3. System PATH
=========================================================Plus=header=end*/

#ifndef _SOLUM_DYNAMIC_LOADER_H
#define _SOLUM_DYNAMIC_LOADER_H

// Solum types/enums/callbacks (header-only, no link dependency)
#include <solum_def.h>
#include <solum_cb.h>

#include <string>
#include <Windows.h>

// Struct definitions from solum.h (needed for direct field access)
typedef struct _CusConnectionParams
{
    const char* ipAddress;
    unsigned int port;
    long long int networkId;
} CusConnectionParams;

typedef struct _CusInitParams
{
    struct Args { int argc; char** argv; } args;
    const char* storeDir;
    CusConnectFn connectFn;
    CusCertFn certFn;
    CusPowerDownFn powerDownFn;
    CusImagingFn imagingFn;
    CusButtonFn buttonFn;
    CusErrorFn errorFn;
    CusElementTestFn elemTestFn;
    CusNewProcessedImageFn newProcessedImageFn;
    CusNewRawImageFn newRawImageFn;
    CusNewSpectralImageFn newSpectralImageFn;
    CusNewImuPortFn newImuPortFn;
    CusNewImuDataFn newImuDataFn;
    int width;
    int height;
} CusInitParams;

//----------------------------------------------------------------------------
// Function pointer typedefs for all Solum API functions
//----------------------------------------------------------------------------

// Lifecycle
typedef int (*pfn_solumInit)(const CusInitParams*);
typedef CusInitParams (*pfn_solumDefaultInitParams)(void);
typedef int (*pfn_solumDestroy)(void);

// Connection
typedef CusConnectionParams (*pfn_solumDefaultConnectionParams)(void);
typedef int (*pfn_solumConnect)(const CusConnectionParams*);
typedef int (*pfn_solumDisconnect)(void);
typedef int (*pfn_solumIsConnected)(void);
typedef int (*pfn_solumSetCert)(const char*);

// Firmware & Updates
typedef int (*pfn_solumFwVersion)(CusPlatform, char*, int);
typedef int (*pfn_solumSoftwareUpdate)(const char*, CusSwUpdateFn, CusProgressFn, int);

// Probe discovery & application
typedef int (*pfn_solumProbes)(CusListFn);
typedef int (*pfn_solumApplications)(const char*, CusListFn);
typedef int (*pfn_solumLoadApplication)(const char*, const char*);

// Probe info & status
typedef int (*pfn_solumStatusInfo)(CusStatusInfo*);
typedef int (*pfn_solumProbeInfo)(CusProbeInfo*);
typedef int (*pfn_solumSetProbeSettings)(const CusProbeSettings*);

// Imaging control
typedef int (*pfn_solumSetOutputSize)(int, int);
typedef int (*pfn_solumSeparateOverlays)(int);
typedef int (*pfn_solumRun)(int);
typedef int (*pfn_solumIsImaging)(void);
typedef int (*pfn_solumPowerDown)(void);
typedef int (*pfn_solumSetMode)(CusMode);
typedef CusMode (*pfn_solumGetMode)(void);
typedef int (*pfn_solumSetFormat)(CusImageFormat);

// Parameters
typedef int (*pfn_solumSetParam)(CusParam, double);
typedef double (*pfn_solumGetParam)(CusParam);
typedef int (*pfn_solumGetRange)(CusParam, CusRange*);
typedef int (*pfn_solumSetTgc)(const CusTgc*);
typedef int (*pfn_solumGetTgc)(CusTgc*);

// ROI & Gate
typedef int (*pfn_solumGetActiveRegion)(double*, int);
typedef int (*pfn_solumGetRoi)(double*, int);
typedef int (*pfn_solumAdjustRoi)(int, int, CusRoiFunction);
typedef int (*pfn_solumMaximizeRoi)(void);
typedef int (*pfn_solumGetGate)(CusGateLines*);
typedef int (*pfn_solumAdjustGate)(int, int);

// Hardware control
typedef int (*pfn_solumEnable5v)(int);
typedef int (*pfn_solumOptimizeWifi)(CusWifiOpt);
typedef int (*pfn_solumResetProbe)(CusProbeReset);

// Raw data
typedef int (*pfn_solumRawDataAvailability)(CusRawAvailabilityFn);
typedef int (*pfn_solumRequestRawData)(long long int, long long int, int, CusRawRequestFn);
typedef int (*pfn_solumReadRawData)(void**, CusRawFn, CusProgressFn);

// Low-level parameters
typedef int (*pfn_solumSetLowLevelParam)(const char*, double);
typedef int (*pfn_solumEnableLowLevelParam)(const char*, int);
typedef int (*pfn_solumSetLowLevelPulse)(const char*, const char*);
typedef double (*pfn_solumGetLowLevelParam)(const char*);

// Safety & diagnostics
typedef int (*pfn_solumGetAcousticIndices)(CusAcoustic*);
typedef int (*pfn_solumBatteryHealth)(CusBatteryHealthFn);
typedef int (*pfn_solumCalibrateImu)(int, CusImuCalibrationFn, CusProgressFn);

// TEE
typedef int (*pfn_solumSetTeeFn)(CusTeeConnectFn);
typedef int (*pfn_solumSetTeeExamInfo)(const char*, const char*, const char*);

//----------------------------------------------------------------------------
// SolumDynLoader: runtime loader for solum.dll
//----------------------------------------------------------------------------
class SolumDynLoader
{
public:
  // Load solum.dll at runtime. Returns true on success.
  static bool Load(const char* dllPath = nullptr)
  {
    if (hModule) return true; // already loaded

    // Search order: explicit path, env var, app dir, system PATH
    if (dllPath && dllPath[0])
    {
      hModule = LoadLibraryA(dllPath);
    }
    if (!hModule)
    {
      char envPath[MAX_PATH] = {};
      if (GetEnvironmentVariableA("SOLUM_DLL_PATH", envPath, MAX_PATH) > 0)
      {
        hModule = LoadLibraryA(envPath);
      }
    }
    if (!hModule)
    {
      hModule = LoadLibraryA("solum.dll");
    }

    if (!hModule)
    {
      lastError = "Failed to load solum.dll";
      return false;
    }

    // Resolve all function pointers
    #define LOAD_FN(name) \
      p_##name = (pfn_##name)GetProcAddress(hModule, #name); \
      if (!p_##name) { missingFns += std::string(#name) + " "; }

    std::string missingFns;

    // Lifecycle
    LOAD_FN(solumInit);
    LOAD_FN(solumDefaultInitParams);
    LOAD_FN(solumDestroy);

    // Connection
    LOAD_FN(solumDefaultConnectionParams);
    LOAD_FN(solumConnect);
    LOAD_FN(solumDisconnect);
    LOAD_FN(solumIsConnected);
    LOAD_FN(solumSetCert);

    // Firmware
    LOAD_FN(solumFwVersion);
    LOAD_FN(solumSoftwareUpdate);

    // Probe discovery & application
    LOAD_FN(solumProbes);
    LOAD_FN(solumApplications);
    LOAD_FN(solumLoadApplication);

    // Probe info & status
    LOAD_FN(solumStatusInfo);
    LOAD_FN(solumProbeInfo);
    LOAD_FN(solumSetProbeSettings);

    // Imaging control
    LOAD_FN(solumSetOutputSize);
    LOAD_FN(solumSeparateOverlays);
    LOAD_FN(solumRun);
    LOAD_FN(solumIsImaging);
    LOAD_FN(solumPowerDown);
    LOAD_FN(solumSetMode);
    LOAD_FN(solumGetMode);
    LOAD_FN(solumSetFormat);

    // Parameters
    LOAD_FN(solumSetParam);
    LOAD_FN(solumGetParam);
    LOAD_FN(solumGetRange);
    LOAD_FN(solumSetTgc);
    LOAD_FN(solumGetTgc);

    // ROI & Gate
    LOAD_FN(solumGetActiveRegion);
    LOAD_FN(solumGetRoi);
    LOAD_FN(solumAdjustRoi);
    LOAD_FN(solumMaximizeRoi);
    LOAD_FN(solumGetGate);
    LOAD_FN(solumAdjustGate);

    // Hardware control
    LOAD_FN(solumEnable5v);
    LOAD_FN(solumOptimizeWifi);
    LOAD_FN(solumResetProbe);

    // Raw data
    LOAD_FN(solumRawDataAvailability);
    LOAD_FN(solumRequestRawData);
    LOAD_FN(solumReadRawData);

    // Low-level parameters
    LOAD_FN(solumSetLowLevelParam);
    LOAD_FN(solumEnableLowLevelParam);
    LOAD_FN(solumSetLowLevelPulse);
    LOAD_FN(solumGetLowLevelParam);

    // Safety & diagnostics
    LOAD_FN(solumGetAcousticIndices);
    LOAD_FN(solumBatteryHealth);
    LOAD_FN(solumCalibrateImu);

    // TEE
    LOAD_FN(solumSetTeeFn);
    LOAD_FN(solumSetTeeExamInfo);

    #undef LOAD_FN

    if (!missingFns.empty())
    {
      // Non-fatal: some functions may not exist in older SDK versions
      lastError = "Optional functions not found: " + missingFns;
    }

    // Core functions must be present
    if (!p_solumInit || !p_solumDestroy || !p_solumConnect ||
        !p_solumDisconnect || !p_solumIsConnected)
    {
      lastError = "Critical solum functions missing from DLL";
      Unload();
      return false;
    }

    return true;
  }

  static void Unload()
  {
    if (hModule)
    {
      FreeLibrary(hModule);
      hModule = nullptr;
    }
  }

  static bool IsLoaded() { return hModule != nullptr; }
  static const std::string& GetLastError() { return lastError; }

  // Check if a specific function is available (for version-dependent features)
  static bool HasFunction(const char* name)
  {
    if (!hModule) return false;
    return GetProcAddress(hModule, name) != nullptr;
  }

  // --- Function pointers (public for macro access) ---

  // Lifecycle
  static inline pfn_solumInit p_solumInit = nullptr;
  static inline pfn_solumDefaultInitParams p_solumDefaultInitParams = nullptr;
  static inline pfn_solumDestroy p_solumDestroy = nullptr;

  // Connection
  static inline pfn_solumDefaultConnectionParams p_solumDefaultConnectionParams = nullptr;
  static inline pfn_solumConnect p_solumConnect = nullptr;
  static inline pfn_solumDisconnect p_solumDisconnect = nullptr;
  static inline pfn_solumIsConnected p_solumIsConnected = nullptr;
  static inline pfn_solumSetCert p_solumSetCert = nullptr;

  // Firmware
  static inline pfn_solumFwVersion p_solumFwVersion = nullptr;
  static inline pfn_solumSoftwareUpdate p_solumSoftwareUpdate = nullptr;

  // Probe discovery & application
  static inline pfn_solumProbes p_solumProbes = nullptr;
  static inline pfn_solumApplications p_solumApplications = nullptr;
  static inline pfn_solumLoadApplication p_solumLoadApplication = nullptr;

  // Probe info & status
  static inline pfn_solumStatusInfo p_solumStatusInfo = nullptr;
  static inline pfn_solumProbeInfo p_solumProbeInfo = nullptr;
  static inline pfn_solumSetProbeSettings p_solumSetProbeSettings = nullptr;

  // Imaging control
  static inline pfn_solumSetOutputSize p_solumSetOutputSize = nullptr;
  static inline pfn_solumSeparateOverlays p_solumSeparateOverlays = nullptr;
  static inline pfn_solumRun p_solumRun = nullptr;
  static inline pfn_solumIsImaging p_solumIsImaging = nullptr;
  static inline pfn_solumPowerDown p_solumPowerDown = nullptr;
  static inline pfn_solumSetMode p_solumSetMode = nullptr;
  static inline pfn_solumGetMode p_solumGetMode = nullptr;
  static inline pfn_solumSetFormat p_solumSetFormat = nullptr;

  // Parameters
  static inline pfn_solumSetParam p_solumSetParam = nullptr;
  static inline pfn_solumGetParam p_solumGetParam = nullptr;
  static inline pfn_solumGetRange p_solumGetRange = nullptr;
  static inline pfn_solumSetTgc p_solumSetTgc = nullptr;
  static inline pfn_solumGetTgc p_solumGetTgc = nullptr;

  // ROI & Gate
  static inline pfn_solumGetActiveRegion p_solumGetActiveRegion = nullptr;
  static inline pfn_solumGetRoi p_solumGetRoi = nullptr;
  static inline pfn_solumAdjustRoi p_solumAdjustRoi = nullptr;
  static inline pfn_solumMaximizeRoi p_solumMaximizeRoi = nullptr;
  static inline pfn_solumGetGate p_solumGetGate = nullptr;
  static inline pfn_solumAdjustGate p_solumAdjustGate = nullptr;

  // Hardware control
  static inline pfn_solumEnable5v p_solumEnable5v = nullptr;
  static inline pfn_solumOptimizeWifi p_solumOptimizeWifi = nullptr;
  static inline pfn_solumResetProbe p_solumResetProbe = nullptr;

  // Raw data
  static inline pfn_solumRawDataAvailability p_solumRawDataAvailability = nullptr;
  static inline pfn_solumRequestRawData p_solumRequestRawData = nullptr;
  static inline pfn_solumReadRawData p_solumReadRawData = nullptr;

  // Low-level parameters
  static inline pfn_solumSetLowLevelParam p_solumSetLowLevelParam = nullptr;
  static inline pfn_solumEnableLowLevelParam p_solumEnableLowLevelParam = nullptr;
  static inline pfn_solumSetLowLevelPulse p_solumSetLowLevelPulse = nullptr;
  static inline pfn_solumGetLowLevelParam p_solumGetLowLevelParam = nullptr;

  // Safety & diagnostics
  static inline pfn_solumGetAcousticIndices p_solumGetAcousticIndices = nullptr;
  static inline pfn_solumBatteryHealth p_solumBatteryHealth = nullptr;
  static inline pfn_solumCalibrateImu p_solumCalibrateImu = nullptr;

  // TEE
  static inline pfn_solumSetTeeFn p_solumSetTeeFn = nullptr;
  static inline pfn_solumSetTeeExamInfo p_solumSetTeeExamInfo = nullptr;

private:
  static inline HMODULE hModule = nullptr;
  static inline std::string lastError;
};

//----------------------------------------------------------------------------
// Macro redirections: existing code calling solumXxx() is transparently
// redirected through the dynamic loader function pointers.
//----------------------------------------------------------------------------

// Lifecycle
#define solumInit             SolumDynLoader::p_solumInit
#define solumDefaultInitParams SolumDynLoader::p_solumDefaultInitParams
#define solumDestroy          SolumDynLoader::p_solumDestroy

// Connection
#define solumDefaultConnectionParams SolumDynLoader::p_solumDefaultConnectionParams
#define solumConnect          SolumDynLoader::p_solumConnect
#define solumDisconnect       SolumDynLoader::p_solumDisconnect
#define solumIsConnected      SolumDynLoader::p_solumIsConnected
#define solumSetCert          SolumDynLoader::p_solumSetCert

// Firmware
#define solumFwVersion        SolumDynLoader::p_solumFwVersion
#define solumSoftwareUpdate   SolumDynLoader::p_solumSoftwareUpdate

// Probe discovery & application
#define solumProbes           SolumDynLoader::p_solumProbes
#define solumApplications     SolumDynLoader::p_solumApplications
#define solumLoadApplication  SolumDynLoader::p_solumLoadApplication

// Probe info & status
#define solumStatusInfo       SolumDynLoader::p_solumStatusInfo
#define solumProbeInfo        SolumDynLoader::p_solumProbeInfo
#define solumSetProbeSettings SolumDynLoader::p_solumSetProbeSettings

// Imaging control
#define solumSetOutputSize    SolumDynLoader::p_solumSetOutputSize
#define solumSeparateOverlays SolumDynLoader::p_solumSeparateOverlays
#define solumRun              SolumDynLoader::p_solumRun
#define solumIsImaging        SolumDynLoader::p_solumIsImaging
#define solumPowerDown        SolumDynLoader::p_solumPowerDown
#define solumSetMode          SolumDynLoader::p_solumSetMode
#define solumGetMode          SolumDynLoader::p_solumGetMode
#define solumSetFormat        SolumDynLoader::p_solumSetFormat

// Parameters
#define solumSetParam         SolumDynLoader::p_solumSetParam
#define solumGetParam         SolumDynLoader::p_solumGetParam
#define solumGetRange         SolumDynLoader::p_solumGetRange
#define solumSetTgc           SolumDynLoader::p_solumSetTgc
#define solumGetTgc           SolumDynLoader::p_solumGetTgc

// ROI & Gate
#define solumGetActiveRegion  SolumDynLoader::p_solumGetActiveRegion
#define solumGetRoi           SolumDynLoader::p_solumGetRoi
#define solumAdjustRoi        SolumDynLoader::p_solumAdjustRoi
#define solumMaximizeRoi      SolumDynLoader::p_solumMaximizeRoi
#define solumGetGate          SolumDynLoader::p_solumGetGate
#define solumAdjustGate       SolumDynLoader::p_solumAdjustGate

// Hardware control
#define solumEnable5v         SolumDynLoader::p_solumEnable5v
#define solumOptimizeWifi     SolumDynLoader::p_solumOptimizeWifi
#define solumResetProbe       SolumDynLoader::p_solumResetProbe

// Raw data
#define solumRawDataAvailability SolumDynLoader::p_solumRawDataAvailability
#define solumRequestRawData   SolumDynLoader::p_solumRequestRawData
#define solumReadRawData      SolumDynLoader::p_solumReadRawData

// Low-level parameters
#define solumSetLowLevelParam    SolumDynLoader::p_solumSetLowLevelParam
#define solumEnableLowLevelParam SolumDynLoader::p_solumEnableLowLevelParam
#define solumSetLowLevelPulse    SolumDynLoader::p_solumSetLowLevelPulse
#define solumGetLowLevelParam    SolumDynLoader::p_solumGetLowLevelParam

// Safety & diagnostics
#define solumGetAcousticIndices  SolumDynLoader::p_solumGetAcousticIndices
#define solumBatteryHealth       SolumDynLoader::p_solumBatteryHealth
#define solumCalibrateImu        SolumDynLoader::p_solumCalibrateImu

// TEE
#define solumSetTeeFn            SolumDynLoader::p_solumSetTeeFn
#define solumSetTeeExamInfo      SolumDynLoader::p_solumSetTeeExamInfo

#endif // _SOLUM_DYNAMIC_LOADER_H
