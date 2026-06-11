/* dualGNSS
* Copyright (C) 2026  D. Gamble (Github: Cyberslug)
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* UbxFullSetup
*
* Configures a u-blox GNSS module from scratch and prints position data
* to the Serial monitor.
*
* The hardware generation is declared at construction so the library can
* select the correct configuration path and parser without probing the
* module at run time.  For auto-detection omit the GpsProvider argument:
*   UbxGNSS gnss;
*
* Wiring (adjust pin numbers to match your hardware):
*   Module TX  →  ESP32 rxPin (11)
*   Module RX  →  ESP32 txPin (12)
*
* Do NOT call Serial1.begin() before gnss.begin().  The library manages
* the serial port internally.
*
*/

#include <Arduino.h>
#include <UbxGNSS.hpp>

static constexpr int8_t RX_PIN = 11;
static constexpr int8_t TX_PIN = 12;

// ---------------------------------------------------------------------------
// Global object — initialise with the known hardware generation.
// Change to UBX_M6_MINUS or UBX_M7_M8 if your module is not M9/M10.
// Omit the argument entirely (UbxGNSS gnss;) to let begin() auto-detect
// the generation via MON-VER.
// ---------------------------------------------------------------------------

UbxGNSS gnss(GpsProvider::UBX_M9_PLUS);

/**
 * @brief Arduino entry point — runs once at power-on or after reset.
 * @details Attempts a fast-track passive start first (200 ms window): if the
 *          module is already configured and outputting frames at the target baud
 *          rate, beginPassive() is sufficient and the full configuration is skipped.
 *          Falls through to gnss.begin() if no valid frames arrive in the window.
 */
void setup()
{
  Serial.begin(115200);
  while (!Serial) {}

  delay(2000);  // Long delay on start to allow CDC Serial to begin functioning

  Serial.println("Configuring GNSS module...");

  // Fast-track: attempt passive start at 115 200 baud (the library's target rate).
  // If the module is already configured and outputting valid frames, this
  // succeeds immediately and the full configuration sequence is unnecessary.
  gnss.beginPassive(Serial1, RX_PIN, TX_PIN);

  const uint32_t deadline = millis() + 200UL;
  while (millis() < deadline) {
    gnss.update();
    if (gnss.hasNewData()) {
      Serial.println("Fast track configure succeeded.");
      return;  // receiving valid frames — no configuration needed
    }
  }

  // Full configuration: sweep baud rates, identify module, apply settings.
  if (!gnss.begin(Serial1, RX_PIN, TX_PIN)) {
    const UbxConfigResult& r = gnss.ubxConfigResult();
    Serial.printf("Configuration failed  status=%u  protocolVersion=%u\n",
                  static_cast<unsigned>(r.status),
                  static_cast<unsigned>(r.protocolVersion));
    while (true) {}  // Halt — inspect Serial monitor for details.
  }

  Serial.printf("Configuration OK  generation=%u  protocolVersion=%u\n",
                static_cast<unsigned>(gnss.getDetectedProvider()),
                static_cast<unsigned>(gnss.ubxConfigResult().protocolVersion));
}

/**
 * @brief Arduino main loop — called repeatedly after setup() returns.
 * @details Calls gnss.update() every iteration to keep the parser fed.  Prints
 *          up to 10 valid position fixes to the Serial monitor; continues calling
 *          update() silently after the limit is reached.
 */
void loop()
{
  static uint8_t count = 0U;
  gnss.update();

  if (gnss.hasNewData() && (count < 10U)) {
    if (gnss.isFixValid()) {
      CrsfGpsPayload p;
      gnss.getPayload(p);

      Serial.printf("Lat %10.7f  Lon %11.7f  Alt %5dm  "
                    "Spd %5.1f km/h  Hdg %6.2f°  Sats %u\n",
                    p.latitude    / 1.0e7,
                    p.longitude   / 1.0e7,
                    static_cast<int>(p.altitude) - 1000,
                    p.groundspeed / 100.0,
                    p.heading     / 100.0,
                    static_cast<unsigned>(p.satellites));
      count++;
    } else {
      Serial.println("Waiting for fix...");
    }
  }
}
