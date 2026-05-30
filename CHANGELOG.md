# Changelog

All notable changes to **SpineUS Server** (internal build name: VerdurePLUS) are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/), and this project adheres to [Semantic Versioning](https://semver.org/) (MAJOR.MINOR.PATCH). No dates are embedded in version numbers.

- **MAJOR** - incompatible changes to config XML schema or device protocol
- **MINOR** - new functionality, backwards compatible
- **PATCH** - backwards-compatible bug fixes and small improvements

Base: PLUS Toolkit 2.9.0 (Queen's University PerkLab). All changes live on the `verdure` branch of the private forks at `github.com/verdure-imaging-inc/` (PlusLib, PlusApp, PlusBuild).

---

## [1.1.2] - 2026-05-30

### Fixed
- **Certificate renewal now runs before joining the probe WiFi.** The probe is its own WiFi access point; on a PC without wired internet, joining it drops the internet route, so renewal against the Clarius Cloud API could never succeed post-connection. The cert expiry is now read directly from the local `.pem` (Windows CryptoAPI) at the start of connection, and renewal happens while internet is still available. The fresh certificate is used for the current session. (PlusLib)

## [1.1.1] - 2026-05-30

### Fixed
- **Certificate auto-renewal timing.** The renewal check ran before the certificate-validation callback populated the days-remaining value, so it never triggered. Moved the check to after probe connection completes. Auto-renewal now fires correctly when the certificate is within the `CertAutoRenewDays` threshold. (PlusLib)
- Installer EXE icon (`MUI_ICON`/`MUI_UNICON`) now embedded in the NSIS package; previously only the header/sidebar bitmaps were branded and the installer file showed the default icon. (Build)

## [1.1.0] - 2026-05-30

### Added
- **Clarius certificate auto-renewal.** On probe connection, the server checks the certificate expiry and, if within the renewal threshold, automatically pulls a fresh certificate from the Clarius Cloud API (scoped to the connected probe''s serial number via `?serial=`) and saves it to `PathToCert`. New XML attributes: `OEMApiKey`, `CertAutoRenewDays` (default 30). Warning logs fire at 30 and 14 days remaining; renewal status is logged on connection. (PlusLib)
- Certificate configuration (`OEMApiKey` SET/NOT SET, `CertAutoRenewDays`) now reported in the server settings log on startup. (PlusLib)

## [1.0.5] - 2026-05-29

### Added
- **`BandwidthOptimization` XML attribute** (default FALSE). Exposes the Solum SDK `bandwidthOptimization` setting. When FALSE, prevents the SDK from auto-downgrading imaging parameters on low WiFi bandwidth, eliminating periodic frame-rate drops during scanning. (PlusLib)

### Changed
- **Full SpineUS rebrand.** All user-visible "Plus" branding removed: installer (`SpineUS-Server-1.0.5-Win64.exe`), launcher EXE (now `SpineUSServer.exe`), window title, tray, icons, license agreement, and shortcuts all rebranded to SpineUS / Verdure Imaging.

---

## [1.0.4] - 2026-05-13

### Added
- **`OptimizeWifiChannel` XML attribute** (default FALSE). Gates the Solum `solumOptimizeWifi()` call so WiFi channel optimization is opt-in. Prevents channel-switch disconnects on connection. (PlusLib)
- **`PowerOffOnDisconnect` XML attribute** (default FALSE). When FALSE, the probe stays powered with WiFi alive between sessions for faster reconnect. (PlusLib)

### Notes
- Stable release. Retains all 1.0.2 BLE fixes; 1.0.3 reconnection changes reverted.

---

## [1.0.3] - 2026-05-12 [REVERTED]

### Changed
- Increased max reconnection attempts 10 to 30, reduced reconnection delay 3000ms to 2000ms, added late-connection handler.

### Reverted
- Late-connection handler caused a blocking-sleep bug. Reverted to 1.0.2 reconnection behavior in 1.0.4.

---

## [1.0.2] - 2026-05-11

### Added
- BLE connection timeout increased 1s to 10s with exponential backoff. (PlusLib)
- Safe BLE retry with `ForceUnpair`. (PlusLib)
- `IntentionalDisconnect` guard for auto-reconnect. (PlusLib)

---

## [1.0.1] - 2026-05-10

### Fixed
- Minor stability fixes following the 1.0.0 release.

---

## [1.0.0] - 2026-05-10

Initial Verdure release. 19 improvements over upstream Queen''s PLUS Toolkit 2.9.0.

### Added
- Dynamic Solum SDK loader (runtime LoadLibrary, eliminates firmware mismatch crashes). (PlusLib)
- Dynamic Motive API loader (Motive 3.0.x and 3.1+ with auto version detection). (PlusLib)
- Three-tier Motive auto-detect: running Motive to NatNet attach; headless API; US-only fallback. (PlusLib)
- Identity transform fallback at 30Hz for US-only mode to keep buffers synced. (PlusLib)
- WiFi auto-reconnect with configurable retry count. (PlusLib)
- `Enable5v` accessory power tied to imaging state. (PlusLib)
- Fan and charger probe status exposed as frame fields. (PlusLib)
- Tracker buffer pre-fill to prevent frame drops. (PlusLib)
- Verdure branding in launcher title bar and installer vendor. (PlusApp)

### Fixed
- Motive DLL search path (SetCurrentDirectory/SetDllDirectory). (PlusLib)
- `AttachToRunningMotive` parser accepts TRUE/FALSE/-1/AUTO. (PlusLib)
- `DeInitializeOEM` null pointer guard. (PlusLib)
- Launcher: grayed connect button, process leak, UI freeze, startup race. (PlusApp)
- VTK module init macros for rendering factories. (PlusApp)

### Build
- `VTK_VERSIONED_INSTALL=OFF` for NSIS packaging. (PlusBuild)
- CMake 4.x compatibility with 3.28 retained. (PlusBuild)

---

## Versioning & Release Process

1. Make changes in `D:\PlusToolkit\PlusLib` and/or `D:\PlusToolkit\PlusApp` (the `verdure` branch).
2. Bump the version string in `PlusServerLauncher/PlusServerLauncherMainWindow.cxx` (title + tray tooltip).
3. Add a new section to this CHANGELOG under the new version number.
4. Commit with a descriptive hyphenated message (e.g. `add-BandwidthOptimization-XML-option-v1.0.5`).
5. Push to the `verdure` remote: `git push verdure verdure`.
6. Tag the release: `git tag -a v1.0.5 -m "SpineUS Server 1.0.5"` then `git push verdure v1.0.5`.
7. Pull into the build tree, build, brand (CPackConfig + NSIS), package.
