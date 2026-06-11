# dualGNSS

Arduino library for **CASIC** and **u-blox M6–M10** GNSS modules on ESP32.
Provides auto-baud detection, complete module configuration, and binary
protocol parsing through two separate concrete classes — **`UbxGNSS`** and
**`CasicGNSS`** — so that a project using only one protocol never links any
object code from the other.

> **ESP32 only.**  The library uses the four-argument
> `HardwareSerial::begin(baud, config, rxPin, txPin)` overload specific to
> the ESP32 Arduino core.

---

## File map

| File | Purpose |
|------|---------|
| `GnssParserBase.hpp/.cpp` | Shared base: output state, read interface, LE field accessors |
| `UbxParser.hpp/.cpp` | UBX frame parser (M6- epoch assembly; M7/M8/M9/M10 NAV-PVT) |
| `CasicParser.hpp/.cpp` | CASIC frame parser (NAV-PV) |
| `UbxGNSS.hpp/.cpp` | Top-level class for u-blox M6–M10 |
| `CasicGNSS.hpp/.cpp` | Top-level class for CASIC modules |
| `UbxConfigurator.hpp/.cpp` | u-blox M6–M10 configurator |
| `CasicConfigurator.hpp/.cpp` | CASIC configurator |
| `UbxMessageBuilder.hpp` | UBX frame builder, header-only |
| `CasicMessageBuilder.hpp` | CASIC frame builder, header-only |
| `CommonParserConstants.hpp` | Constants shared by both protocol stacks |
| `UbxConstants.hpp` | All u-blox protocol constants |
| `CasicConstants.hpp` | All CASIC protocol constants |
| `CommonStructures.hpp` | `CrsfGpsPayload` struct |
| `GpsProvider.hpp` | `GpsProvider` enum — hardware generation selector for `UbxGNSS` |

---

## Architecture

```
         ┌─────────────────────────────────────────────────────────┐
         │                     GnssParserBase                      │
         │  m_serial  m_payload  m_newData  m_fixValid             │
         │  hasNewData()  isFixValid()  getPayload()               │
         │  readU1/U4/I4/R4/R8  clampAltU16  normaliseHeading1e5   │
         └────────────────────────┬────────────────────────────────┘
                                  │ (inheritance, no virtual functions)
               ┌──────────────────┴──────────────────┐
               │                                     │
       ┌───────┴────────┐                   ┌────────┴────────┐
       │   UbxParser    │                   │   CasicParser   │
       │  feedByte()    │                   │  feedByte()     │
       │  M7/M8/M9/M10  │                   │  NAV-PV         │
       │    NAV-PVT     │                   │                 │
       │  M6- 3-message │                   │                 │
       │  epoch assembly│                   │                 │
       └───────┬────────┘                   └────────┬────────┘
               │ (held by value)                     │ (held by value)
       ┌───────┴────────┐                   ┌────────┴────────┐
       │    UbxGNSS     │                   │   CasicGNSS     │
       │  begin()       │                   │  begin()        │
       │  beginPassive()│                   │  beginPassive() │
       │  update() ...  │                   │  update() ...   │
       └────────────────┘                   └─────────────────┘
```

`UbxConfigurator` and `CasicConfigurator` are instantiated on the stack
inside `begin()` and destroyed on return; they are not held as members of
the GNSS classes.

Each concrete parser lives in its own translation unit.  A project that
includes only `UbxGNSS.hpp` will never link `CasicParser.cpp` or
`CasicGNSS.cpp`, and vice versa.

---

## GpsProvider

`GpsProvider` is passed to the `UbxGNSS` constructor to declare the
hardware generation.  The library uses this to select the correct
configuration path and message parser without probing the module at
run time.  `UNKNOWN` triggers automatic detection via MON-VER during
`begin()`.

| Value | Hardware | Configuration path | Parser message set |
|-------|----------|-------------------|--------------------|
| `UBX_M6_MINUS` | u-blox M6 and below | Legacy CFG-PRT / CFG-RATE / CFG-MSG | NAV-POSLLH + NAV-SOL + NAV-VELNED |
| `UBX_M7_M8` | u-blox M7 / M8 | Legacy CFG-PRT / CFG-RATE / CFG-MSG | NAV-PVT |
| `UBX_M9_PLUS` | u-blox M9 / M10 (incl. ZED-F9P) | CFG-VALSET / CFG-VALGET | NAV-PVT |
| `UNKNOWN` | Any — auto-detect | Resolved via MON-VER in `begin()` | Resolved at run time |

`UNKNOWN` is **not valid** for `beginPassive()` — pass a specific generation
to the constructor when using passive mode.  `GpsProvider` is not used by
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
    CrsfGpsPayload p;
    gnss.getPayload(p);
    Serial.printf("Lat %.7f  Lon %.7f  Alt %dm\n",
      p.latitude  / 1e7,
      p.longitude / 1e7,
      static_cast<int>(p.altitude) - 1000);
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
void getPayload(CrsfGpsPayload& dest) const;

bool                     isConfigured()      const;
const CasicConfigResult& casicConfigResult() const;
```

### `CrsfGpsPayload`

CRSF (Crossfire Serial Protocol) is the telemetry format used by ELRS and
TBS Crossfire systems.  All fields use CRSF-native scaling so they can be
copied directly into a CRSF GPS frame without further conversion.

| Field | Type | Units |
|-------|------|-------|
| `latitude` | `int32_t` | degrees × 10 000 000 (1e-7 °) |
| `longitude` | `int32_t` | degrees × 10 000 000 (1e-7 °) |
| `groundspeed` | `uint16_t` | km/h × 100 (hundredths of km/h) |
| `heading` | `uint16_t` | degrees × 100 (hundredths of °), range [0, 36 000) |
| `altitude` | `uint16_t` | metres + 1000 CRSF offset (sea level = 1000) |
| `satellites` | `uint8_t` | satellites used in fix |

