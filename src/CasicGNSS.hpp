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

#pragma once

// ---------------------------------------------------------------------------
// CasicGNSS — top-level Arduino interface for CASIC GNSS modules.
//
// Two operating modes
// -------------------
//
// Full mode — begin()
//   Configures the module from scratch: auto-detects the current baud rate,
//   enables CASIC binary output, sets the navigation rate, changes the baud
//   rate to 115 200, and saves settings to non-volatile storage.
//   Do NOT call serial.begin() before calling begin().
//
//     CasicGNSS gnss;
//     if (!gnss.begin(Serial1, 11, 12)) {
//       // inspect gnss.casicConfigResult() for details
//     }
//
// Passive mode — beginPassive()
//   Skips configuration and goes straight to parsing.  The library opens
//   the serial port internally — do NOT call serial.begin() beforehand.
//   Use when the module has already been configured (e.g. by a previous
//   begin() call).
//
//     CasicGNSS gnss;
//     gnss.beginPassive(Serial1, 11, 12);         // TARGET_BAUD_RATE baud default
//     gnss.beginPassive(Serial1, 11, 12, 9600);   // explicit baud rate
//
// Main loop
// ---------
//     gnss.update();
//     if (gnss.hasNewData() && gnss.isFixValid()) {
//       CrsfGpsPayload p;         // CRSF-encoded output (legacy - deprecated)
//       gnss.getPayload(p);
//
//       GnssData d;               // Natural-unit output
//       gnss.getData(d);
//     }
//
// IMPORTANT — ESP32 only
//   Uses the four-argument HardwareSerial::begin(baud, config, rxPin, txPin)
//   overload specific to the ESP32 Arduino core.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include "CasicParser.hpp"
#include "CasicConfigurator.hpp"

class CasicGNSS {
public:

  /**
   * @brief Constructor.  Initialises all members to safe defaults.
   */
  CasicGNSS();

  /**
   * @brief Full mode: runs the configurator then initialises the parser.
   * @details Executes the complete Phase 0 / 1 / 2 / 3 sequence via CasicConfigurator.
   *          The serial port is managed internally — do NOT call serial.begin() first.
   *          On failure the port is left open at 115 200 baud for diagnostic queries.
   * @param serial Reference to the HardwareSerial port connected to the CASIC module.
   * @param rxPin  MCU pin connected to module TX (module → MCU).
   * @param txPin  MCU pin connected to module RX (MCU → module).
   * @return true on success; false on failure — call casicConfigResult() for details.
   */
  bool begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin);

  /**
   * @brief Passive mode: opens the serial port and initialises the parser.
   * @details Skips all configuration; the module must already be set up correctly.
   *          The serial port is managed internally — do NOT call serial.begin() first.
   * @param serial Reference to the HardwareSerial port connected to the module.
   * @param rxPin  MCU pin connected to module TX (module → MCU).
   * @param txPin  MCU pin connected to module RX (MCU → module).
   * @param baud   Baud rate the module is already operating at; defaults to TARGET_BAUD_RATE.
   */
  void beginPassive(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                    uint32_t baud = CasicConfigurator::TARGET_BAUD_RATE);

  /**
   * @brief Reads available bytes and advances the CASIC frame parser.
   * @details Delegates to CasicParser::update().  Does nothing if begin() or
   *          beginPassive() has not been called.  Call every iteration of loop().
   */
  void update();

  /**
   * @brief Returns true exactly once per newly assembled navigation solution.
   * @return true if a new solution has arrived since the last call, false otherwise.
   */
  bool hasNewData();

  /**
   * @brief Returns true when the most recently decoded solution has a valid 3-D fix.
   * @return true if the last decoded fix was a valid 3-D GNSS fix.
   */
  bool isFixValid() const;

  /**
   * @brief Copies the most recently decoded navigation solution into dest.
   * @param dest Reference to a CrsfGpsPayload struct that receives the current solution.
   */
  void getPayload(CrsfGpsPayload& dest) const;

  /**
   * @brief Copies the most recently decoded navigation solution into dest in
   *        natural / SI units without any protocol-specific conversion.
   * @details Delegates unconditionally to the parser.  Fields not yet populated
   *          by the active parser are zero.  The contents of dest are undefined
   *          until at least one complete solution has been assembled.
   * @param dest Reference to a GnssData struct that receives the current solution.
   */
  void getData(GnssData& dest) const;

  // -------------------------------------------------------------------------
  // Diagnostics
  // -------------------------------------------------------------------------

  /**
   * @brief Returns true after a successful begin() call, or after any beginPassive() call.
   * @return true if the parser is active and update() will process incoming bytes.
   */
  bool isConfigured() const;

  /**
   * @brief Returns the result struct from the most recent full-mode configuration attempt.
   * @details All fields are zero-initialised after a beginPassive() call.
   * @return Const reference to the stored CasicConfigResult.
   */
  const CasicConfigResult& casicConfigResult() const;

private:
  CasicParser       m_parser;          ///< Concrete CASIC frame parser instance (held by value).
  CasicConfigResult m_configureResult; ///< Result from the most recent begin() call.
  bool              m_configured;      ///< true after a successful begin() or any beginPassive() call.
};
