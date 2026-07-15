# dualGNSS

Arduino library for **CASIC** and **u-blox M6–M10** GNSS modules on ESP32.
Provides auto-baud detection, complete module configuration, and binary
protocol parsing through two separate concrete classes — **`UbxGNSS`** and
**`CasicGNSS`** — so that a project using only one protocol never links any
object code from the other. Note that a combined or Universal GNSS interface
has been added so that unified code for either module type can be written
see the UniversalFullSetup.ino example

> **ESP32 only.**  The library uses the four-argument
> `HardwareSerial::begin(baud, config, rxPin, txPin)` overload specific to
> the ESP32 Arduino core.

---

## File map

| File | Purpose |
|------|---------|
| `dualGNSS.hpp` | Primary interface to the library provides a template class for a universal GNSS interface |
| `GnssParserBase.hpp/.cpp` | Shared base: output state, read interface, LE field accessors |
| `UbxParser.hpp/.cpp` | UBX frame parser (M6- epoch assembly; M7/M8/M9/M10 NAV-PVT) |
| `CasicParser.hpp/.cpp` | CASIC frame parser (NAV-PV, NAV-TIMEUTC) |
| `UbxGNSS.hpp/.cpp` | Top-level class for u-blox M6–M10 |
| `CasicGNSS.hpp/.cpp` | Top-level class for CASIC modules |
| `UbxConfigurator.hpp/.cpp` | u-blox M6–M10 configurator |
| `CasicConfigurator.hpp/.cpp` | CASIC configurator |
| `UbxMessageBuilder.hpp` | UBX frame builder, header-only |
| `CasicMessageBuilder.hpp` | CASIC frame builder, header-only |
| `CommonParserConstants.hpp` | Protocol-neutral constants shared by both stacks |
| `CrsfConstants.hpp` | CRSF-specific encoding constants (altitude offset) |
| `UbxConstants.hpp` | All u-blox protocol constants |
| `CasicConstants.hpp` | All CASIC protocol constants |
| `GnssTypes.hpp` | `GnssData` and `CrsfGpsPayload` structs |
| `GpsProvider.hpp` | `GpsProvider` enum — hardware generation selector for `UbxGNSS` |

---

## Architecture

```
         ┌──────────────────────────────────────────────────────────────────┐
         │                       GnssParserBase                             │
         │  m_serial   m_payload (GnssData)   m_newData                    │
         │  hasNewData()  isFixValid()  getPayload()  getData()             │
         │  readU1/U2/U4/I4/R4/R8   clampAltU16                            │
         │  normaliseHeading1e5   normaliseHeadingRaw1e5                    │
         └────────────────────────────┬─────────────────────────────────────┘
                                      │ (inheritance, no virtual functions)
                   ┌──────────────────┴──────────────────┐
                   │                                     │
         ┌─────────┴────────┐                   ┌────────┴────────┐
         │    UbxParser     │                   │   CasicParser   │
         │  feedByte()      │                   │  feedByte()     │
         │  M7/M8/M9/M10    │                   │  NAV-PV         │
         │    NAV-PVT       │                   │  NAV-TIMEUTC    │
         │  M6- epoch       │                   │                 │
         │  NAV-POSLLH      │                   │                 │
         │  NAV-SOL         │                   │                 │
         │  NAV-VELNED      │                   │                 │
         │  NAV-TIMEUTC     │                   │                 │
         └─────────┬────────┘                   └────────┬────────┘
                   │ (held by value)                     │ (held by value)
         ┌─────────┴────────┐                   ┌────────┴────────┐
         │     UbxGNSS      │                   │   CasicGNSS     │
         │  begin()         │                   │  begin()        │
         │  beginPassive()  │                   │  beginPassive() │
         │  update() ...    │                   │  update() ...   │
         └─────────┬────────┘                   └────────┬────────┘
                   │                                     │
                   └──────────────────┬──────────────────┘
                            ┌─────────┴────────┐
                            │       Gnss       │
                            │  begin()         │
                            │  beginPassive()  │
                            │  update() ...    │
                            └──────────────────┘
```

`UbxConfigurator` and `CasicConfigurator` are instantiated on the stack
inside `begin()` and destroyed on return; they are not held as members of
the GNSS classes.

Each concrete parser lives in its own translation unit.  A project that
includes only `UbxGNSS.hpp` will never link `CasicParser.cpp` or
`CasicGNSS.cpp`, and vice versa.

---

## UbxSeries

`UbxSeries` is passed to the `UbxGNSS` constructor to declare the
hardware generation.  The library uses this to select the correct
configuration path and message parser without probing the module at
run time.  `UNKNOWN` triggers automatic detection via MON-VER during
`begin()`.

| Value | Hardware | Configuration path | Message set |
|-------|----------|--------------------|-------------|
| `UBX_M6_MINUS` | u-blox M6 and below | Legacy CFG-PRT / CFG-RATE / CFG-MSG | NAV-POSLLH + NAV-SOL + NAV-VELNED + NAV-TIMEUTC |
| `UBX_M7_M8` | u-blox M7 / M8 | Legacy CFG-PRT / CFG-RATE / CFG-MSG | NAV-PVT (time included) |
| `UBX_M9_PLUS` | u-blox M9 / M10 (incl. ZED-F9P) | CFG-VALSET / CFG-VALGET | NAV-PVT (time included) |
| `UNKNOWN` | Any — auto-detect | Resolved via MON-VER in `begin()` | Resolved at run time |

`UNKNOWN` is **not valid** for `beginPassive()` — pass a specific generation
to the constructor when using passive mode.  `UbxSeries` is not used by
`CasicGNSS`.

---

## Quick start

### u-blox — full mode, declared generation (recommended)

Declaring the generation at construction skips the MON-VER probe and
goes straight to configuration.  Do **not** call `Serial1.begin()`
before `begin()`.

```cpp
#include "UbxGNSS.hpp"

UbxGNSS gnss(GpsProvider::UBX_M7_M8);  // or UBX_M6_MINUS / UBX_M9_PLUS

void setup() {
  Serial.begin(115200);
  if (!gnss.begin(Serial1, 11, 12)) {
    Serial.println("Configuration failed");
    while (true) {}
  }
}

void loop() {
  gnss.update();
  if (gnss.hasNewData() && gnss.isFixValid()) {

    // Natural-unit output
    GnssData d;
    gnss.getData(d);
    Serial.printf("Lat %.7f  Lon %.7f  Alt MSL %.1f m  Speed %.1f km/h\n",
      d.latitude   / 1.0e7,
      d.longitude  / 1.0e7,
      d.altMSL     / 1000.0,
      d.gSpeed     * 3.6 / 1000.0);

    // legacy CRSF-encoded output — for ELRS / Crossfire telemetry will be removed
    CrsfGpsPayload p;
    gnss.getPayload(p);
    // p can be copied directly into a CRSF 0x02 GPS frame
  }
}
```

### u-blox — full mode, auto-detect generation

Omit the constructor argument (or pass `GpsProvider::UNKNOWN`) and the
library identifies the module via MON-VER.  Use this when the hardware
generation is not known at compile time.

```cpp
UbxGNSS gnss;  // equivalent to UbxGNSS gnss(GpsProvider::UNKNOWN)

void setup() {
  if (!gnss.begin(Serial1, 11, 12)) { /* ... */ }
  // After a successful begin(), gnss.getDetectedProvider() returns
  // the resolved UBX_M6_MINUS / UBX_M7_M8 / UBX_M9_PLUS value.
}
```

### u-blox — passive mode (module already configured)

The library opens the serial port internally.  Do **not** call
`Serial1.begin()` before `beginPassive()`.

```cpp
#include "UbxGNSS.hpp"

UbxGNSS gnss(GpsProvider::UBX_M7_M8);  // UNKNOWN is not valid here

void setup() {
  gnss.beginPassive(Serial1, 11, 12);          // 115 200 baud (default)
  // gnss.beginPassive(Serial1, 11, 12, 9600); // explicit baud rate
}
```

### CASIC — full mode

```cpp
#include "CasicGNSS.hpp"

CasicGNSS gnss;

void setup() {
  Serial.begin(115200);
  if (!gnss.begin(Serial1, 11, 12)) {
    Serial.println("Configuration failed");
    while (true) {}
  }
}

void loop() {
  gnss.update();
  if (gnss.hasNewData() && gnss.isFixValid()) {
    GnssData d;
    gnss.getData(d);
    // Check time validity before using date/time fields
    if ((d.validFlags & GNSS_FLAG_TIME_VALID) != 0) {
      Serial.printf("UTC %04u-%02u-%02u %02u:%02u:%02u.%03u\n",
        d.year, d.month, d.day, d.hour, d.minute, d.second, d.millisecond);
    }
  }
}
```

### CASIC — passive mode (module already configured)

The library opens the serial port internally.  Do **not** call
`Serial1.begin()` before `beginPassive()`.

```cpp
#include "CasicGNSS.hpp"

CasicGNSS gnss;

void setup() {
  gnss.beginPassive(Serial1, 11, 12);          // 115 200 baud (default)
  // gnss.beginPassive(Serial1, 11, 12, 9600); // explicit baud rate
}
```

---

## API reference

### `UbxGNSS`

```cpp
// Constructor — declare the hardware generation, or omit for auto-detect.
explicit UbxGNSS(GpsProvider generation = GpsProvider::UNKNOWN);

// Full mode — do NOT call serial.begin() before this.
// If generation is UNKNOWN, the module is probed via MON-VER.
bool begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin);

// Passive mode — do NOT call serial.begin() before this.
// Pass a specific generation to the constructor; UNKNOWN is not valid here.
void beginPassive(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                  uint32_t baud = 115200UL);

void update();        // call every loop() iteration
bool hasNewData();    // true once per new solution, then resets
bool isFixValid() const;

// Natural-unit output — all fields in SI / protocol-agnostic units.
void getData(GnssData& dest) const;

// legacy CRSF-encoded output — fields in CRSF wire-format scaling.
void getPayload(CrsfGpsPayload& dest) const;

bool                   isConfigured()        const;
GpsProvider            getDetectedProvider() const;  // resolved generation
const UbxConfigResult& ubxConfigResult()     const;
```

### `CasicGNSS`

```cpp
// Full mode — do NOT call serial.begin() before this.
bool begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin);

// Passive mode — do NOT call serial.begin() before this.
void beginPassive(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                  uint32_t baud = 115200UL);

void update();        // call every loop() iteration
bool hasNewData();    // true once per new solution, then resets
bool isFixValid() const;

// Natural-unit output — all fields in SI / protocol-agnostic units.
void getData(GnssData& dest) const;

// Legacy CRSF-encoded output — fields in CRSF wire-format scaling.
void getPayload(CrsfGpsPayload& dest) const;

bool                     isConfigured()      const;
const CasicConfigResult& casicConfigResult() const;
```

---

## Output data

The library exposes two output structs.  Both are populated from the same
internal `GnssData` store; `getPayload()` converts to CRSF units on the
way out. Note that getPayload() is deprecated and will be removed.

### `GnssData`

Protocol-agnostic navigation solution in natural / SI units.  Written once
per epoch by the active parser.  Fields not yet valid for the active sensor
family are zero; inspect `validFlags` before consuming optional fields.

#### Position

| Field | Type | Unit | Notes |
|-------|------|------|-------|
| `latitude` | `int32_t` | 1e-7 ° | Positive = north |
| `longitude` | `int32_t` | 1e-7 ° | Positive = east |
| `altMSL` | `int32_t` | mm | MSL altitude, signed. Zero = sea level, no offset applied |
| `altEllipsoid` | `int32_t` | mm | WGS84 ellipsoidal height |

#### Velocity — NED convention

Down is positive; a descending vehicle has positive `velD`.
CASIC modules report velocity in ENU; the library negates `velU` before
storing so `velD` is consistent across all sensor families.

| Field | Type | Unit | Notes |
|-------|------|------|-------|
| `velN` | `int32_t` | mm/s | Positive = northward |
| `velE` | `int32_t` | mm/s | Positive = eastward |
| `velD` | `int32_t` | mm/s | Positive = descending |
| `gSpeed` | `int32_t` | mm/s | 2-D ground speed, always ≥ 0 |
| `headMot` | `int32_t` | 1e-5 ° | Heading of motion, normalised to [0, 36 000 000) |

#### Accuracy estimates — 1-sigma

All accuracy fields are stored as 1-sigma linear estimates regardless of
sensor family.  CASIC modules publish accuracy as variances (m², (m/s)²,
°²); the parser applies `sqrtf()` before storing so all fields carry the
same meaning.

| Field | Type | Unit |
|-------|------|------|
| `hAcc` | `uint32_t` | mm |
| `vAcc` | `uint32_t` | mm |
| `sAcc` | `uint32_t` | mm/s |
| `headAcc` | `uint32_t` | 1e-5 ° |
| `pDOP` | `uint16_t` | dimensionless × 100 (e.g. 185 = DOP 1.85) |

#### Fix status

| Field | Type | Notes |
|-------|------|-------|
| `fixType` | `uint8_t` | 0 = no fix, 1 = dead reckoning, 2 = 2-D, 3 = 3-D, 4 = GNSS + DR |
| `validFlags` | `uint8_t` | Bitmask — see table below |
| `satellites` | `uint8_t` | Satellites used in the navigation solution |

`isFixValid()` returns true when `GNSS_FLAG_FIX_OK` is set; it is a
convenience wrapper around `validFlags`.

#### `validFlags` bitmask

| Constant | Bit | Meaning |
|----------|-----|---------|
| `GNSS_FLAG_FIX_OK` | 0 | A valid 3-D GNSS fix has been obtained |
| `GNSS_FLAG_VEL_VALID` | 1 | Velocity fields (velN/E/D, gSpeed, headMot) are valid |
| `GNSS_FLAG_DATE_VALID` | 2 | UTC date fields (year, month, day) are valid |
| `GNSS_FLAG_TIME_VALID` | 3 | UTC time fields (hour, minute, second, millisecond) are valid |

#### UTC time

Time fields are updated from a separate NAV-TIMEUTC message on M6- and CASIC
modules; on M7+ and later they come from NAV-PVT in the same epoch.
All time fields are zero until the module achieves a time fix.
Check `GNSS_FLAG_DATE_VALID` and `GNSS_FLAG_TIME_VALID` before use.

| Field | Type | Range | Notes |
|-------|------|-------|-------|
| `year` | `uint16_t` | 1999–2099 | 0 = not yet valid |
| `month` | `uint8_t` | 1–12 | |
| `day` | `uint8_t` | 1–31 | |
| `hour` | `uint8_t` | 0–23 | |
| `minute` | `uint8_t` | 0–59 | |
| `second` | `uint8_t` | 0–60 | 60 during a positive leap second |
| `millisecond` | `uint16_t` | 0–999 | |

#### Field availability by sensor family

| Field group | CASIC | UBX M6- | UBX M7/M8 | UBX M9/M10 |
|-------------|-------|---------|-----------|------------|
| Position, velocity, fix status | ✓ | ✓ | ✓ | ✓ |
| Accuracy estimates (hAcc…pDOP) | ✓ | ✓ | ✓ | ✓ |
| UTC time | ✓ via NAV-TIMEUTC | ✓ via NAV-TIMEUTC | ✓ from NAV-PVT | ✓ from NAV-PVT |

> **CASIC altitude note.**  Some CASIC firmware versions store geoid separation
> with the opposite sign convention from the geodetic standard, making the
> effective MSL formula `height + sepGeoid` rather than `height − sepGeoid`.
> If the reported `altMSL` differs from a known MSL benchmark by approximately
> twice the local geoid undulation (typically 30–50 m at mid-latitudes), edit
> the sign on the `altMSL` line in `CasicParser.cpp`.

---

### `CrsfGpsPayload`

CRSF (Crossfire Serial Protocol) is the telemetry format used by ELRS and
TBS Crossfire systems.  All fields use CRSF-native scaling so they can be
copied directly into a CRSF 0x02 GPS frame without further conversion.
Produced by `getPayload()`; derived from the same internal `GnssData` store
as `getData()`.

| Field | Type | Units |
|-------|------|-------|
| `latitude` | `int32_t` | degrees × 10 000 000 (1e-7 °) |
| `longitude` | `int32_t` | degrees × 10 000 000 (1e-7 °) |
| `groundspeed` | `uint16_t` | km/h × 100 (hundredths of km/h) |
| `heading` | `uint16_t` | degrees × 100 (hundredths of °), range [0, 36 000) |
| `altitude` | `uint16_t` | metres + 1000 CRSF offset (sea level = 1000) |
| `satellites` | `uint8_t` | satellites used in fix |
