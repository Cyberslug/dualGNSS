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
// UbxGNSS — top-level Arduino interface for u-blox M6–M10 GNSS modules.
//
// Hardware generation
// -------------------
// Pass the module generation to the constructor so the library can select
// the correct configurator path and parser without any run-time detection:
//
//   UbxGNSS gnss(GpsProvider::UBX_M6_MINUS);   // M6 and below
//   UbxGNSS gnss(GpsProvider::UBX_M7_M8);      // M7 / M8
//   UbxGNSS gnss(GpsProvider::UBX_M9_PLUS);    // M9 / M10
//
// If the generation is not known at compile time, omit the argument (or pass
// GpsProvider::UNKNOWN explicitly) and begin() will detect it via MON-VER:
//
//   UbxGNSS gnss;                               // auto-detect in begin()
//
// UNKNOWN is not valid for beginPassive() — a run-time assertion fires if
// it is used there.
//
// Two operating modes
// -------------------
//
// Full mode — begin()
//   Configures the module from scratch: auto-detects the current baud rate,
//   enables UBX binary output, sets the navigation rate, enables the correct
//   message type for the detected or declared generation, changes the baud
//   rate to 115 200, and saves to all non-volatile storage.
//   Do NOT call serial.begin() before calling begin().
//
//     UbxGNSS gnss(GpsProvider::UBX_M7_M8);
//     if (!gnss.begin(Serial1, 11, 12)) {
//       // inspect gnss.ubxConfigResult() for details
//     }
//
// Passive mode — beginPassive()
//   Skips configuration and goes straight to parsing.  The library opens
//   the serial port internally — do NOT call serial.begin() beforehand.
//   The generation passed to the constructor is used directly; UNKNOWN is
//   not permitted and will trigger an assertion.
//
//     UbxGNSS gnss(GpsProvider::UBX_M7_M8);
//     gnss.beginPassive(Serial1, 11, 12);          // TARGET_BAUD_RATE baud default
//     gnss.beginPassive(Serial1, 11, 12, 9600);    // explicit baud rate
//
// Main loop
// ---------
//     gnss.update();
//     if (gnss.hasNewData() && gnss.isFixValid()) {
//       CrsfGpsPayload p;
//       gnss.getPayload(p);
//     }
//
// IMPORTANT — ESP32 only
//   Uses the four-argument HardwareSerial::begin(baud, config, rxPin, txPin)
//   overload specific to the ESP32 Arduino core.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <assert.h>
#include "GpsProvider.hpp"
#include "UbxParser.hpp"
#include "UbxConfigurator.hpp"

class UbxGNSS {
public:

  /**
   * @brief Constructor.
   * @details UNKNOWN triggers MON-VER auto-detection during begin() and is the
   *          default.  A specific generation must be supplied when using beginPassive().
   * @param generation Hardware generation of the connected module; defaults to UNKNOWN.
   */
  explicit UbxGNSS(GpsProvider generation = GpsProvider::UNKNOWN);

  /**
   * @brief Full mode: runs the configurator then initialises the parser.
   * @details Executes the complete Phase 0 / 1 / 2 sequence via UbxConfigurator.
   *          The serial port is managed internally — do NOT call serial.begin() first.
   *          If the constructor generation was UNKNOWN, the detected generation is
   *          stored and available via getDetectedProvider() after a successful call.
   *          On failure the port is left open at TARGET_BAUD_RATE baud for diagnostic queries.
   * @param serial Reference to the HardwareSerial port connected to the u-blox module.
   * @param rxPin  MCU pin connected to module TX (module → MCU).
   * @param txPin  MCU pin connected to module RX (MCU → module).
   * @return true on success; false on failure — call ubxConfigResult() for details.
   */
  bool begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin);

  /**
   * @brief Passive mode: opens the serial port and initialises the parser.
   * @details Skips all configuration; the module must already be set up correctly.
   *          The serial port is managed internally — do NOT call serial.begin() first.
   *          A specific generation (not UNKNOWN) must have been given to the constructor;
   *          a run-time assertion fires if UNKNOWN is used here.
   * @param serial Reference to the HardwareSerial port connected to the module.
   * @param rxPin  MCU pin connected to module TX (module → MCU).
   * @param txPin  MCU pin connected to module RX (MCU → module).
   * @param baud   Baud rate the module is already operating at; defaults to TARGET_BAUD_RATE.
   */
  void beginPassive(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                    uint32_t baud = UbxConfigurator::TARGET_BAUD_RATE);

  /**
   * @brief Reads available bytes and advances the UBX frame parser.
   * @details Delegates to UbxParser::update().  Does nothing if begin() or
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

  // -------------------------------------------------------------------------
  // Diagnostics
  // -------------------------------------------------------------------------

  /**
   * @brief Returns true after a successful begin() call, or after any beginPassive() call.
   * @return true if the parser is active and update() will process incoming bytes.
   */
  bool isConfigured() const;

  /**
   * @brief Returns the hardware generation currently in use by the parser.
   * @details In full mode with UNKNOWN constructor argument, reflects the generation
   *          detected via MON-VER.  In all other cases, reflects the constructor value.
   * @return The active GpsProvider generation enumerator.
   */
  GpsProvider getDetectedProvider() const;

  /**
   * @brief Returns the result struct from the most recent full-mode configuration attempt.
   * @details All fields are zero-initialised after a beginPassive() call.
   * @return Const reference to the stored UbxConfigResult.
   */
  const UbxConfigResult& ubxConfigResult() const;

private:
  UbxParser       m_parser;          ///< Concrete UBX frame parser instance (held by value).
  UbxConfigResult m_configureResult; ///< Result from the most recent begin() call.
  GpsProvider     m_generation;      ///< Generation set at construction; may be updated by begin().
  bool            m_configured;      ///< true after a successful begin() or any beginPassive() call.
};
