# Verdure PLUS XML Configuration Options

Reference for all Verdure-specific XML attributes in the Clarius OEM and OptiTrack device configurations.

---

## Clarius OEM Device (VideoDevice)

### Connection Options

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| OptimizeWifiChannel | BOOL | FALSE | When TRUE, optimizes WiFi channel on connect. Can cause disconnects. Keep FALSE for stability. |
| PowerOffOnDisconnect | BOOL | FALSE | When TRUE, powers off probe on PLUS disconnect. When FALSE, probe stays on with WiFi. |

### Probe Settings

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| ProbeSerialNum | STRING | (required) | Probe serial number |
| PathToCert | STRING | (required) | Path to probe certificate PEM file |
| ProbeType | STRING | (required) | Probe model (e.g. C3HD3) |
| ImagingApplication | STRING | abdomen | Imaging application preset |
| Enable5v | BOOL | FALSE | Enable 5V accessory power on probe |
| EnablePenetrationMode | BOOL | FALSE | Enable deep penetration mode |
| EnableAutoGain | BOOL | FALSE | Enable automatic gain |
| EnableAutoFocus | BOOL | FALSE | Enable automatic focus |

### Power Management

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| KeepAwakeCharging | BOOL | FALSE | Keep probe awake on charger |
| KeepAwakeTimeoutMin | INT | 60 | Minutes before sleep (0 = never) |
| StationaryTimeoutSec | INT | 0 | No-motion sleep timeout (0 = disabled) |
| WakeOnShake | BOOL | FALSE | Wake probe on motion |
| AutoFreezeTimeoutSec | INT | 0 | Auto-freeze after idle (0 = disabled) |
| ContactDetectionTimeoutSec | INT | 0 | Stop on no skin contact (0 = disabled) |
### Imaging Parameters

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| DepthMm | INT | - | Imaging depth in mm |
| GainPercent | INT | - | Gain percentage |
| DynRangePercent | INT | - | Dynamic range percentage |
| ImagingMode | STRING | BMode | Imaging mode |
| FrameSize | STRING | 512 512 1 | Frame dimensions |

### Button Configuration

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| UpButtonMode | ENUM | USER | FREEZE, USER, or DISABLED |
| DownButtonMode | ENUM | USER | FREEZE, USER, or DISABLED |

### Other

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| FreezeOnPoorWifiSignal | BOOL | FALSE | Freeze imaging on poor WiFi |
| PowerButtonsEnabled | BOOL | TRUE | Enable probe power buttons |
| SoundEnabled | BOOL | TRUE | Enable probe sounds |

---

## OptiTrack Device (TrackerDevice)

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| AttachToRunningMotive | STRING | -1 | TRUE = NatNet, FALSE = headless, -1/AUTO = auto-detect |
| MotiveProfile | STRING | (required) | Path to .motive profile |
| MotiveCalibration | STRING | (required) | Path to .cal calibration |

### Auto-Detect Tiers (AttachToRunningMotive="-1")

| Tier | Condition | Behavior |
|------|-----------|----------|
| 1 | Motive.exe running | Attach via NatNet |
| 2 | Motive closed, API available | Headless API mode |
| 3 | API not available | US-only, identity transforms at 30Hz |