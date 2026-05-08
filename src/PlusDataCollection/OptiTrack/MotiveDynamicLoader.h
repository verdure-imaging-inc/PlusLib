/*=Plus=header=begin======================================================
    Program: Plus
    Copyright (c) Verdure Imaging Inc, Stockton, California. All rights reserved.
    See License.txt for details.

    Dynamic loader for OptiTrack Motive API - enables a single PLUS binary
    to work with Motive 3.0.x, 3.1.x, and 3.3.x at runtime without
    rebuilding. Also provides auto-detection of running Motive instances.

    USAGE:
    Replace the Motive API includes and #if MOTIVE_VERSION blocks with:
      #include "MotiveDynamicLoader.h"
    Call MotiveDynLoader::Load() in InternalConnect()
    Call MotiveDynLoader::Unload() in InternalDisconnect()
    Use MotiveDynLoader::IsMotiveRunning() for auto-detection.
=========================================================Plus=header=end*/

#ifndef _MOTIVE_DYNAMIC_LOADER_H
#define _MOTIVE_DYNAMIC_LOADER_H

#include <string>
#include <Windows.h>
#include <TlHelp32.h>

// NatNet includes (always available, version-stable)
#include <NatNetClient.h>
#include <NatNetTypes.h>

//----------------------------------------------------------------------------
// Motive API version enumeration
//----------------------------------------------------------------------------
enum class MotiveVersion
{
  Unknown = 0,
  V3_0,    // Motive 3.0.x - TT_* exports, eMotiveAPIResult return type
  V3_1     // Motive 3.1+ (incl 3.3.x) - MotiveAPI:: namespace, eResult return type
};

//----------------------------------------------------------------------------
// Mangled export names for each Motive version
// These are MSVC x64 decorated names extracted via dumpbin /exports
//----------------------------------------------------------------------------
namespace MotiveExports
{
  // Motive 3.0.x (TT_* prefix)
  namespace V3_0
  {
    static const char* Initialize       = "?TT_Initialize@@YA?AW4eMotiveAPIResult@@XZ";
    static const char* Shutdown         = "?TT_Shutdown@@YA?AW4eMotiveAPIResult@@XZ";
    static const char* TestConnection   = "?TT_TestSoftwareMutex@@YA?AW4eMotiveAPIResult@@XZ";
    static const char* Update           = "?TT_Update@@YA?AW4eMotiveAPIResult@@XZ";
    static const char* LoadProfile      = "?TT_LoadProfile@@YA?AW4eMotiveAPIResult@@PEB_W@Z";
    static const char* LoadCalibration  = "?TT_LoadCalibration@@YA?AW4eMotiveAPIResult@@PEB_WPEAH@Z";
    static const char* StreamNP         = "?TT_StreamNP@@YA?AW4eMotiveAPIResult@@_N@Z";
    static const char* AddRigidBodies   = "?TT_AddRigidBodies@@YA?AW4eMotiveAPIResult@@PEB_W@Z";
    static const char* CameraCount      = "?TT_CameraCount@@YAHXZ";
    static const char* CameraName       = "?TT_CameraName@@YA_NHPEA_WH@Z";
    static const char* RigidBodyCount   = "?TT_RigidBodyCount@@YAHXZ";
    static const char* RigidBodyName    = "?TT_RigidBodyName@@YA_NHPEA_WH@Z";
  }

  // Motive 3.1+ / 3.3.x (MotiveAPI:: namespace)
  namespace V3_1
  {
    static const char* Initialize       = "?Initialize@MotiveAPI@@YA?AW4eResult@1@XZ";
    static const char* Shutdown         = "?Shutdown@MotiveAPI@@YAXXZ";
    static const char* TestConnection   = "?CanConnectToDevices@MotiveAPI@@YA_NXZ";
    static const char* Update           = "?Update@MotiveAPI@@YA?AW4eResult@1@XZ";
    static const char* LoadProfile      = "?LoadProfile@MotiveAPI@@YA?AW4eResult@1@PEB_W@Z";
    static const char* LoadCalibration  = "?LoadCalibration@MotiveAPI@@YA?AW4eResult@1@PEB_WPEAH@Z";
    static const char* StreamNP         = "?StreamNP@MotiveAPI@@YA?AW4eResult@1@_N@Z";
    static const char* AddRigidBodies   = "?AddRigidBodies@MotiveAPI@@YA?AW4eResult@1@PEB_W@Z";
    static const char* CameraCount      = "?CameraCount@MotiveAPI@@YAHXZ";
    // CameraName not yet confirmed for 3.1+ - resolve at load time
    static const char* CameraName       = nullptr;
    static const char* RigidBodyCount   = "?RigidBodyCount@MotiveAPI@@YAHXZ";
    static const char* RigidBodyName    = "?RigidBodyName@MotiveAPI@@YA_NHPEA_WH@Z";
    static const char* MapToResultStr   = "?MapToResultString@MotiveAPI@@YA?BV?$basic_string@_WU?$char_traits@_W@std@@V?$allocator@_W@2@@std@@W4eResult@1@@Z";
  }
}

//----------------------------------------------------------------------------
// Unified function pointer types
// All result-returning functions use int (0 = success) for uniformity
//----------------------------------------------------------------------------
typedef int  (*pfn_MotiveResultVoid)(void);       // Initialize, Update, Shutdown(3.0)
typedef void (*pfn_MotiveVoid)(void);              // Shutdown(3.1+)
typedef bool (*pfn_MotiveBoolVoid)(void);           // CanConnectToDevices(3.1+)
typedef int  (*pfn_MotiveResultWstr)(const wchar_t*);  // LoadProfile, AddRigidBodies
typedef int  (*pfn_MotiveResultWstrPInt)(const wchar_t*, int*); // LoadCalibration
typedef int  (*pfn_MotiveResultBool)(bool);         // StreamNP
typedef int  (*pfn_MotiveIntVoid)(void);            // CameraCount, RigidBodyCount
typedef bool (*pfn_MotiveBoolIntWstrInt)(int, wchar_t*, int); // CameraName, RigidBodyName

//----------------------------------------------------------------------------
// MotiveDynLoader: runtime loader for MotiveAPI.dll with auto-detection
//----------------------------------------------------------------------------
class MotiveDynLoader
{
public:
  //------------------------------------------------------------------------
  // Check if Motive.exe is currently running (for auto-detect mode)
  //------------------------------------------------------------------------
  static bool IsMotiveRunning()
  {
    bool found = false;
    HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snapshot == INVALID_HANDLE_VALUE) return false;

    PROCESSENTRY32 entry;
    entry.dwSize = sizeof(PROCESSENTRY32);
    if (Process32First(snapshot, &entry))
    {
      do
      {
        if (_stricmp(entry.szExeFile, "Motive.exe") == 0)
        {
          found = true;
          break;
        }
      } while (Process32Next(snapshot, &entry));
    }
    CloseHandle(snapshot);
    return found;
  }

  //------------------------------------------------------------------------
  // Load MotiveAPI.dll and resolve function pointers
  // dllPath: explicit path, or nullptr to search standard locations
  //------------------------------------------------------------------------
  static bool Load(const char* dllPath = nullptr)
  {
    if (hModule) return true;

    // Search order: explicit path, Motive install dir, app dir, PATH
    if (dllPath && dllPath[0])
    {
      hModule = LoadLibraryA(dllPath);
    }

    if (!hModule)
    {
      // Try standard Motive install location
      hModule = LoadLibraryA("C:\\Program Files\\OptiTrack\\Motive\\lib\\MotiveAPI.dll");
    }
    if (!hModule)
    {
      hModule = LoadLibraryA("MotiveAPI.dll");
    }
    if (!hModule)
    {
      lastError = "Failed to load MotiveAPI.dll";
      return false;
    }

    // Detect version by probing for 3.1+ namespace symbols first
    detectedVersion = MotiveVersion::Unknown;
    FARPROC testV31 = GetProcAddress(hModule, MotiveExports::V3_1::Initialize);
    FARPROC testV30 = GetProcAddress(hModule, MotiveExports::V3_0::Initialize);

    if (testV31)
    {
      detectedVersion = MotiveVersion::V3_1;
      ResolveV31();
    }
    else if (testV30)
    {
      detectedVersion = MotiveVersion::V3_0;
      ResolveV30();
    }
    else
    {
      lastError = "MotiveAPI.dll loaded but no recognized API symbols found";
      Unload();
      return false;
    }

    // Verify critical functions resolved
    if (!p_Initialize || !p_Update || !p_StreamNP)
    {
      lastError = "Critical Motive API functions could not be resolved";
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
    detectedVersion = MotiveVersion::Unknown;
    ClearPointers();
  }

  static bool IsLoaded() { return hModule != nullptr; }
  static MotiveVersion GetVersion() { return detectedVersion; }
  static const std::string& GetLastError() { return lastError; }

  //------------------------------------------------------------------------
  // Unified API - hides version differences behind a clean interface
  // All return int: 0 = success, non-zero = failure
  //------------------------------------------------------------------------

  static int Initialize()
  {
    return p_Initialize ? p_Initialize() : -1;
  }

  static void Shutdown()
  {
    if (detectedVersion == MotiveVersion::V3_1 && p_Shutdown_void)
      p_Shutdown_void();
    else if (p_Shutdown_int)
      p_Shutdown_int();
  }

  static bool CanConnectToDevices()
  {
    if (detectedVersion == MotiveVersion::V3_1 && p_CanConnect_bool)
      return p_CanConnect_bool();
    else if (p_CanConnect_int)
      return p_CanConnect_int() == 0; // 3.0: 0 = success = mutex free
    return false;
  }

  static int Update()
  {
    return p_Update ? p_Update() : -1;
  }

  static int LoadProfile(const wchar_t* filename)
  {
    return p_LoadProfile ? p_LoadProfile(filename) : -1;
  }

  static int LoadCalibration(const wchar_t* filename, int* cameraCount = nullptr)
  {
    return p_LoadCalibration ? p_LoadCalibration(filename, cameraCount) : -1;
  }

  static int StreamNP(bool enable)
  {
    return p_StreamNP ? p_StreamNP(enable) : -1;
  }

  static int AddRigidBodies(const wchar_t* filename)
  {
    return p_AddRigidBodies ? p_AddRigidBodies(filename) : -1;
  }

  static int CameraCount()
  {
    return p_CameraCount ? p_CameraCount() : 0;
  }

  static bool CameraName(int index, wchar_t* name, int nameLen)
  {
    return p_CameraName ? p_CameraName(index, name, nameLen) : false;
  }

  static int RigidBodyCount()
  {
    return p_RigidBodyCount ? p_RigidBodyCount() : 0;
  }

  static bool RigidBodyName(int index, wchar_t* name, int nameLen)
  {
    return p_RigidBodyName ? p_RigidBodyName(index, name, nameLen) : false;
  }

private:
  //------------------------------------------------------------------------
  // Resolve functions for Motive 3.1+ (MotiveAPI:: namespace)
  //------------------------------------------------------------------------
  static void ResolveV31()
  {
    p_Initialize     = (pfn_MotiveResultVoid)GetProcAddress(hModule, MotiveExports::V3_1::Initialize);
    p_Shutdown_void  = (pfn_MotiveVoid)GetProcAddress(hModule, MotiveExports::V3_1::Shutdown);
    p_Shutdown_int   = nullptr;
    p_CanConnect_bool = (pfn_MotiveBoolVoid)GetProcAddress(hModule, MotiveExports::V3_1::TestConnection);
    p_CanConnect_int  = nullptr;
    p_Update         = (pfn_MotiveResultVoid)GetProcAddress(hModule, MotiveExports::V3_1::Update);
    p_LoadProfile    = (pfn_MotiveResultWstr)GetProcAddress(hModule, MotiveExports::V3_1::LoadProfile);
    p_LoadCalibration = (pfn_MotiveResultWstrPInt)GetProcAddress(hModule, MotiveExports::V3_1::LoadCalibration);
    p_StreamNP       = (pfn_MotiveResultBool)GetProcAddress(hModule, MotiveExports::V3_1::StreamNP);
    p_AddRigidBodies = (pfn_MotiveResultWstr)GetProcAddress(hModule, MotiveExports::V3_1::AddRigidBodies);
    p_CameraCount    = (pfn_MotiveIntVoid)GetProcAddress(hModule, MotiveExports::V3_1::CameraCount);
    // CameraName: try the known 3.0 mangled name pattern adapted for namespace
    // If not found, it may have a different signature in newer versions
    p_CameraName     = (pfn_MotiveBoolIntWstrInt)GetProcAddress(hModule, "?CameraName@MotiveAPI@@YA_NHPEA_WH@Z");
    p_RigidBodyCount = (pfn_MotiveIntVoid)GetProcAddress(hModule, MotiveExports::V3_1::RigidBodyCount);
    p_RigidBodyName  = (pfn_MotiveBoolIntWstrInt)GetProcAddress(hModule, MotiveExports::V3_1::RigidBodyName);
  }

  //------------------------------------------------------------------------
  // Resolve functions for Motive 3.0.x (TT_* prefix)
  //------------------------------------------------------------------------
  static void ResolveV30()
  {
    p_Initialize     = (pfn_MotiveResultVoid)GetProcAddress(hModule, MotiveExports::V3_0::Initialize);
    p_Shutdown_int   = (pfn_MotiveResultVoid)GetProcAddress(hModule, MotiveExports::V3_0::Shutdown);
    p_Shutdown_void  = nullptr;
    p_CanConnect_int = (pfn_MotiveResultVoid)GetProcAddress(hModule, MotiveExports::V3_0::TestConnection);
    p_CanConnect_bool = nullptr;
    p_Update         = (pfn_MotiveResultVoid)GetProcAddress(hModule, MotiveExports::V3_0::Update);
    p_LoadProfile    = (pfn_MotiveResultWstr)GetProcAddress(hModule, MotiveExports::V3_0::LoadProfile);
    p_LoadCalibration = (pfn_MotiveResultWstrPInt)GetProcAddress(hModule, MotiveExports::V3_0::LoadCalibration);
    p_StreamNP       = (pfn_MotiveResultBool)GetProcAddress(hModule, MotiveExports::V3_0::StreamNP);
    p_AddRigidBodies = (pfn_MotiveResultWstr)GetProcAddress(hModule, MotiveExports::V3_0::AddRigidBodies);
    p_CameraCount    = (pfn_MotiveIntVoid)GetProcAddress(hModule, MotiveExports::V3_0::CameraCount);
    p_CameraName     = (pfn_MotiveBoolIntWstrInt)GetProcAddress(hModule, MotiveExports::V3_0::CameraName);
    p_RigidBodyCount = (pfn_MotiveIntVoid)GetProcAddress(hModule, MotiveExports::V3_0::RigidBodyCount);
    p_RigidBodyName  = (pfn_MotiveBoolIntWstrInt)GetProcAddress(hModule, MotiveExports::V3_0::RigidBodyName);
  }

  static void ClearPointers()
  {
    p_Initialize = nullptr;
    p_Shutdown_void = nullptr;
    p_Shutdown_int = nullptr;
    p_CanConnect_bool = nullptr;
    p_CanConnect_int = nullptr;
    p_Update = nullptr;
    p_LoadProfile = nullptr;
    p_LoadCalibration = nullptr;
    p_StreamNP = nullptr;
    p_AddRigidBodies = nullptr;
    p_CameraCount = nullptr;
    p_CameraName = nullptr;
    p_RigidBodyCount = nullptr;
    p_RigidBodyName = nullptr;
  }

  // Module handle and state
  static inline HMODULE hModule = nullptr;
  static inline MotiveVersion detectedVersion = MotiveVersion::Unknown;
  static inline std::string lastError;

  // Function pointers (version-specific variants for Shutdown/CanConnect)
  static inline pfn_MotiveResultVoid p_Initialize = nullptr;
  static inline pfn_MotiveVoid p_Shutdown_void = nullptr;
  static inline pfn_MotiveResultVoid p_Shutdown_int = nullptr;
  static inline pfn_MotiveBoolVoid p_CanConnect_bool = nullptr;
  static inline pfn_MotiveResultVoid p_CanConnect_int = nullptr;
  static inline pfn_MotiveResultVoid p_Update = nullptr;
  static inline pfn_MotiveResultWstr p_LoadProfile = nullptr;
  static inline pfn_MotiveResultWstrPInt p_LoadCalibration = nullptr;
  static inline pfn_MotiveResultBool p_StreamNP = nullptr;
  static inline pfn_MotiveResultWstr p_AddRigidBodies = nullptr;
  static inline pfn_MotiveIntVoid p_CameraCount = nullptr;
  static inline pfn_MotiveBoolIntWstrInt p_CameraName = nullptr;
  static inline pfn_MotiveIntVoid p_RigidBodyCount = nullptr;
  static inline pfn_MotiveBoolIntWstrInt p_RigidBodyName = nullptr;
};

#endif // _MOTIVE_DYNAMIC_LOADER_H
