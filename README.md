# SpineUS Server (PlusLib)

**Maintained by Verdure Imaging, Inc.** &nbsp;|&nbsp; Stockton, California &nbsp;|&nbsp; [verdureimaging.com](https://www.verdureimaging.com)

The data-acquisition and device-interface library powering **SpineUS**, Verdure Imaging''s FDA 510(k)-cleared platform for radiation-free 3D spine visualization using standard 2D ultrasound with optical tracking.

This repository is Verdure''s production fork of the [PLUS Toolkit](http://www.plustoolkit.org) (Public software Library for UltraSound imaging research, Queen''s University PerkLab). It is maintained on the `verdure` branch and adds hardware integration, stability, and device-management features required by the SpineUS product.

---

## What this fork adds

SpineUS Server extends upstream PLUS 2.9.0 with production features for the Clarius HD3 wireless ultrasound and OptiTrack optical tracking stack:

- **Clarius certificate auto-renewal** — automatic certificate refresh from the Clarius Cloud API before expiration, scoped per probe serial.
- **Dynamic device loading** — runtime loading of the Clarius Solum SDK and OptiTrack Motive API, eliminating firmware/version-mismatch crashes.
- **Three-tier tracking auto-detect** — works whether Motive is running, headless, or unavailable.
- **Connection resilience** — BLE timeout backoff, WiFi auto-reconnect, tracker buffer pre-fill, and configurable bandwidth/WiFi optimization.

See **[CHANGELOG.md](CHANGELOG.md)** for the complete, versioned history.

---

## Versioning

This project follows [Semantic Versioning](https://semver.org/). Releases are tagged `vMAJOR.MINOR.PATCH` (e.g. `v1.1.0`). The current release is documented at the top of [CHANGELOG.md](CHANGELOG.md).

- **MAJOR** — incompatible config-schema or device-protocol changes
- **MINOR** — new backwards-compatible functionality
- **PATCH** — backwards-compatible bug fixes

---

## Repository structure

Verdure maintains three coordinated forks, all on the `verdure` branch:

| Repository | Purpose |
|------------|---------|
| **PlusLib** | Core data acquisition, device interfaces, calibration (this repo) |
| **PlusApp** | SpineUS Server launcher application and UI |
| **PlusBuild** | CMake superbuild that assembles dependencies and produces the installer |

The customized device code lives in `src/PlusDataCollection/Clarius/` and `src/PlusDataCollection/OptiTrack/`.

---

## Build

The library is assembled via the CMake superbuild in the PlusBuild repository, which downloads and configures all dependencies (VTK, ITK, OpenIGTLink, etc.). Production builds target Windows 64-bit. See the PlusBuild repository for full build instructions.

Configurable device options (XML attributes) added by Verdure are documented in `VERDURE_XML_OPTIONS.md`.

---

## Upstream & attribution

This is a derivative work of the PLUS Toolkit, developed by the Laboratory for Percutaneous Surgery (PerkLab) at Queen''s University and contributors. PLUS is distributed under a BSD-style license. Verdure gratefully acknowledges the PLUS project and maintains all original copyright and license notices.

- Upstream project: [PlusToolkit.org](http://www.plustoolkit.org)
- Upstream source: [github.com/PlusToolkit/PlusLib](https://github.com/PlusToolkit/PlusLib)
- The `master` branch of this fork tracks upstream; Verdure changes are isolated on the `verdure` branch.

---

## License

PLUS is distributed under a BSD-style license — see [License.txt](License.txt). Verdure''s additions are subject to the SpineUS Server Software License Agreement included with the distributed installer. SpineUS is a trademark of Verdure Imaging, Inc.

---

## Contact

Verdure Imaging, Inc. — [chris@verdureimaging.com](mailto:chris@verdureimaging.com)
