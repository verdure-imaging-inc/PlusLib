# Verdure PLUS - Changes from Upstream Queen's University PLUS Toolkit

**Version:** VerdurePLUS 1.0.0
**Base:** PLUS Toolkit 2.9.0 (Queen's University PerkLab)
**Date:** May 10, 2026
**Maintainer:** Chris Schlenger, Verdure Imaging (chris@verdureimaging.com)

---

## Overview

Verdure PLUS is a customized build of the PLUS Toolkit optimized for the SpineUS platform. It combines Clarius HD3 wireless ultrasound with OptiTrack optical tracking, adding dynamic device loading, auto-detection, improved stability, and Verdure branding. All changes are on the `verdure` branch of three private GitHub repos at `github.com/verdure-imaging-inc/`.

---

## PlusLib Changes

### 1. Dynamic Solum SDK Loader (SolumDynamicLoader.h)
- Replaced static Clarius Solum SDK linkage with runtime dynamic loading via LoadLibraryA/GetProcAddress.
- Eliminates firmware version mismatch crashes. The correct Solum DLL is loaded at runtime regardless of which version was used at compile time.
- Files: src/PlusDataCollection/Clarius/SolumDynamicLoader.h (new file)

### 2. Dynamic Motive API Loader (MotiveDynamicLoader.h)
- Replaced static NatNet/Motive SDK linkage with runtime dynamic loading. Supports both Motive 3.0.x and 3.1+ APIs with automatic version detection.
- Single binary works with any Motive version. No need to recompile when Motive is updated.
- Files: src/PlusDataCollection/OptiTrack/MotiveDynamicLoader.h (new file)

### 3. Three-Tier Motive Auto-Detect (vtkPlusOptiTrack.cxx)
- AttachToRunningMotive attribute accepts TRUE, FALSE, -1 (auto), or AUTO. Default is -1 (auto-detect). Three-tier logic:
  - Tier 1: Motive.exe is running -> attach via NatNet (like TRUE mode)
  - Tier 2: Motive.exe not running, MotiveAPI.dll available -> load API headlessly (like FALSE mode)
  - Tier 3: MotiveAPI.dll not available -> US-only mode with identity transforms at 30Hz
- One XML config works on any PC regardless of Motive state.
- Files: src/PlusDataCollection/OptiTrack/vtkPlusOptiTrack.cxx

### 4. Motive DLL Search Path Fix (MotiveDynamicLoader.h)
- Uses SetCurrentDirectoryA and SetDllDirectoryA to set the working directory to C:\Program Files\OptiTrack\Motive\ before loading MotiveAPI.dll from the lib\ subfolder.
- MotiveAPI.dll has runtime dependencies (opencv_world454.dll, Qt5Core.dll, tbb.dll, etc.) in the root Motive folder. Without this, LoadLibraryA fails.
- Files: src/PlusDataCollection/OptiTrack/MotiveDynamicLoader.h

### 5. Identity Transform Fallback for US-Only Mode (vtkPlusOptiTrack.cxx)
- When Motive is skipped (Tier 3), enables a 30Hz internal update thread that pushes identity transforms with TOOL_OUT_OF_VIEW status to all tracker tool buffers.
- Keeps tracker buffers synchronized with ultrasound timestamps so the OpenIGTLink output channel doesn't stall.
- Files: src/PlusDataCollection/OptiTrack/vtkPlusOptiTrack.cxx

### 6. AttachToRunningMotive Parser Fix (vtkPlusOptiTrack.cxx)
- Replaced XML_READ_BOOL_ATTRIBUTE_NONMEMBER_REQUIRED (only accepts TRUE/FALSE) with a string-based parser accepting TRUE, FALSE, true, false, 1, 0, -1, AUTO, auto.
- Files: src/PlusDataCollection/OptiTrack/vtkPlusOptiTrack.cxx

### 7. DeInitializeOEM Null Pointer Guard (vtkPlusClariusOEM.cxx)
- Added SolumDynLoader::IsLoaded() check at the top of InternalDisconnect() and DeInitializeOEM().
- Prevents null function pointer crash during device factory enumeration when Solum library hasn't been loaded yet.
- Files: src/PlusDataCollection/Clarius/vtkPlusClariusOEM.cxx

### 8. BLE Connection Improvements (vtkPlusClariusOEM.cxx, ClariusBLE.cxx)
- Improved BLE timeout handling, exponential backoff for retries, event-driven waits instead of sleep-based polling where possible.
- Files: src/PlusDataCollection/Clarius/vtkPlusClariusOEM.cxx, src/PlusDataCollection/Clarius/ClariusBLE.cxx

### 9. WiFi Auto-Reconnect (vtkPlusClariusOEM.cxx)
- Added automatic WiFi reconnection logic with configurable retry count (default 10 attempts).
- Files: src/PlusDataCollection/Clarius/vtkPlusClariusOEM.cxx

### 10. 5V Power Tied to Imaging State (vtkPlusClariusOEM.cxx)
- Enable5v XML parameter controls the probe's 5V accessory power output, activated when imaging starts.
- Files: src/PlusDataCollection/Clarius/vtkPlusClariusOEM.cxx

### 11. Fan and Charger Frame Fields (vtkPlusClariusOEM.cxx)
- Exposes probe fan status and charger connection state as frame fields in the output data stream.
- Files: src/PlusDataCollection/Clarius/vtkPlusClariusOEM.cxx

### 12. WiFi Channel Optimization (vtkPlusClariusOEM.cxx)
- Automatic WiFi channel optimization on probe connection.
- Files: src/PlusDataCollection/Clarius/vtkPlusClariusOEM.cxx

### 13. VTK Rendering Link Dependencies (CMakeLists.txt)
- Added missing VTK rendering module link dependencies for VTK 9.1.
- Files: src/PlusRendering/CMakeLists.txt

---

## PlusApp Changes

### 1. Verdure Branding (PlusServerLauncherMainWindow.cxx, .ui)
- Window title shows "Verdure PLUS Server Launcher - [version]". System tray and menus updated. Installer vendor changed to "Verdure Imaging".
- Files: PlusServerLauncher/PlusServerLauncherMainWindow.cxx, PlusServerLauncher/PlusServerLauncherMainWindow.ui

### 2. VTK Module Init (PlusServerLauncherMain.cxx)
- Added VTK_MODULE_INIT macros for rendering factories. Prevents "no rendering factory" crash on startup with VTK 9.1.
- Files: PlusServerLauncher/PlusServerLauncherMain.cxx

### 3. Launcher Stability Fixes (PlusServerLauncherMainWindow.cxx)
- Fixed grayed-out connect button after disconnect, process handle leak on server restart, UI freeze during server shutdown, and startup race condition.
- Files: PlusServerLauncher/PlusServerLauncherMainWindow.cxx

---

## PlusBuild Changes

### 1. VTK Versioned Install (CMakeLists.txt)
- Set VTK_VERSIONED_INSTALL=OFF so VTK installs to vtk/ instead of vtk-9.1/. Required for NSIS installer packaging.

### 2. CMake 4.x Compatibility
- Fixed compatibility with CMake 4.x while maintaining CMake 3.28 support.

---

## Tested On

- ULTRASOUND Desktop (Windows, Motive installed, OptiTrack cameras connected)
- Razer 18 Laptop (Windows, Motive not installed)
- MSI PC (Windows, Motive installed, OptiTrack cameras connected)
