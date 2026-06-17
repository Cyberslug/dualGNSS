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
#include <dualGNSS.hpp>

static constexpr int8_t RX_PIN = 11;
static constexpr int8_t TX_PIN = 12;
static constexpr GnssType TYPE = GnssType::CASIC;
static constexpr UbxSeries GENERATION = UbxSeries::UBX_M9_PLUS;

class GnssDriver
{
  public:
    bool  begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
                { return m_gnss.begin(serial, rxPin, txPin);       }
    void  beginPassive(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                    uint32_t baud = UbxConfigurator::TARGET_BAUD_RATE)
                    {m_gnss.beginPassive(serial, rxPin, txPin, baud);}
    void  update()       { m_gnss.update();      }
    bool  hasNewData()   { return m_gnss.hasNewData();  }
    bool  isFixValid()   { return m_gnss.isFixValid();  }
    void  getData(GnssData& dest)   { m_gnss.getData(dest);  }
    GnssConfigResult getConfigResult() { return m_gnss.getConfigResult(); }

  private:
    // One line. Config drives both the module type and the protocol version.
    // If GNSS_TYPE is CASIC the second argument is accepted but unused.
    Gnss<TYPE, GENERATION> m_gnss;  // Configured external to the class
};




// ---------------------------------------------------------------------------
// Global object — type and generation selected by constants in configuration data
// ---------------------------------------------------------------------------

GnssDriver gnss;

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
    const GnssConfigResult& r = gnss.getConfigResult();
    switch (TYPE){
      case GnssType::UBX:
        Serial.printf("Configuration failed  status=%u  protocolVersion=%u\n",
                      static_cast<unsigned>(r.status),
                      static_cast<unsigned>(r.protocolVersion));
        break;
      case GnssType::CASIC:
        Serial.printf("Configuration failed status=%u\n", static_cast<unsigned>(r.status));
        break;
      default:
        break;
    }
    while (true) {}  // Halt — inspect Serial monitor for details.
  }
  
  const GnssConfigResult& r = gnss.getConfigResult();
  switch (TYPE){
    case GnssType::UBX:
      Serial.printf("Configuration OK  generation=%u  protocolVersion=%u\n",
                    static_cast<unsigned>(r.detectedProvider),
                    static_cast<unsigned>(r.protocolVersion));
      break;
      case GnssType::CASIC:
      Serial.printf("Configuration OK  status=%u\n",
                    static_cast<unsigned>(r.status));
      break;
    default:
      break;
  }
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
      GnssData d;
      gnss.getData(d);

      Serial.printf("Lat %10.7f  Lon %11.7f  Alt %5.3f m  "
                    "Spd %5.1f km/h  Hdg %6.2f°  Sats %u ",
                    d.latitude    / 1.0e7,
                    d.longitude   / 1.0e7,
                    d.altMSL      / 1000.0,
                    d.gSpeed      * 0.0036,
                    d.headMot     / 1.0e5,
                    static_cast<unsigned>(d.satellites));
      Serial.printf("Raw Alt %d\n", d.altMSL);
      count++;
    } else {
      Serial.println("Waiting for fix...");
    }
  }
}
