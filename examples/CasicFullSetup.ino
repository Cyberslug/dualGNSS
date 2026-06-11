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

/* CasicFullSetup
*
* Configures a CASIC GNSS module from scratch and prints position data
* to the Serial monitor.
*
* Wiring (adjust pin numbers to match your hardware):
*   Module TX  →  ESP32 rxPin (11)
*   Module RX  →  ESP32 txPin (12)
*
* Do NOT call Serial1.begin() before gnss.begin().  The library manages
* the serial port internally.
*
* Altitude note: some CASIC firmware versions store the geoid separation
* with the opposite sign convention.  If the reported altitude is wrong
* by approximately twice the local geoid undulation (typically 30–50 m),
* see the sign-convention comment in CasicParser.cpp processNavPv().
*
*/

#include <Arduino.h>
#include <CasicGNSS.hpp>

static constexpr int8_t RX_PIN = 11;
static constexpr int8_t TX_PIN = 12;

CasicGNSS gnss;

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

  delay(2000U);  // Long delay on start to allow CDC Serial to begin functioning

  Serial.println("Configuring CASIC GNSS module...");

  // Fast-track: attempt passive start at 115 200 baud (the library's target rate).
  gnss.beginPassive(Serial1, RX_PIN, TX_PIN);

  const uint32_t deadline = millis() + 200UL;
  while (millis() < deadline) {
    gnss.update();
    if (gnss.hasNewData()) {
      Serial.println("Fast track configure succeeded.");
      return;  // receiving valid frames, no configuration needed
    }
  }

  // Full configuration: sweep baud rates, verify contact, apply settings.
  if (!gnss.begin(Serial1, RX_PIN, TX_PIN)) {
    const CasicConfigResult& r = gnss.casicConfigResult();
    Serial.printf("Configuration failed  status=%u\n",
                  static_cast<unsigned>(r.status));
    Serial.printf("  observedBaud=%lu  observedProto=0x%02X  observedInterval=%ums\n",
                  r.observedBaudRate,
                  static_cast<unsigned>(r.observedProtoMask),
                  static_cast<unsigned>(r.observedIntervalMs));
    while (true) {}  // Halt — inspect Serial monitor for details.
  }

  Serial.println("Configuration OK.");
}


/**
 * @brief Arduino main loop — called repeatedly after setup() returns.
 * @details Calls gnss.update() every iteration to keep the parser fed.  Prints
 *          up to 10 position reports to the Serial monitor (valid fix or waiting
 *          message); continues calling update() silently after the limit is reached.
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
    } else {
      Serial.println("Waiting for fix...");
    }
    count++;
  }
}
