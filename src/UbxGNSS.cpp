/* DualGNSS
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

#include "UbxGNSS.hpp"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

/**
 * @brief Constructor.  Stores the requested generation and sets m_configured to false.
 * @details The generation is passed through to UbxParser::begin() after a successful
 *          configure() call.  When UNKNOWN, begin() will update m_generation with the
 *          value detected by the configurator via MON-VER.
 */
UbxGNSS::UbxGNSS(GpsProvider generation)
  : m_parser()
  , m_configureResult{}
  , m_generation(generation)
  , m_configured(false)
{
}

// ---------------------------------------------------------------------------
// Full mode
// ---------------------------------------------------------------------------

/**
 * @brief Runs UbxConfigurator::configure(), then initialises the parser on success.
 * @details Resets m_configured and m_configureResult before each attempt so a failed
 *          call leaves a clean diagnostic state.  When UNKNOWN was supplied at
 *          construction, m_generation is updated to the generation detected via MON-VER
 *          so that getDetectedProvider() and getPayload() return consistent values.
 *          The configurator leaves the port open at TARGET_BAUD_RATE on success, so
 *          the parser can begin immediately without reopening the port.
 */
bool UbxGNSS::begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
{
  m_configured      = false;
  m_configureResult = UbxConfigResult{};

  UbxConfigurator cfg;
  m_configureResult = cfg.configure(serial, rxPin, txPin, m_generation);

  if (m_configureResult.status != UbxConfigStatus::OK) {
    return false;
  }

  // If UNKNOWN was used at construction, adopt the generation the configurator
  // detected via MON-VER so it is available via getDetectedProvider().
  if (m_generation == GpsProvider::UNKNOWN) {
    m_generation = m_configureResult.detectedProvider;
  }

  // The configurator leaves the port open at TARGET_BAUD_RATE on success,
  // so the parser can start immediately.
  m_parser.begin(serial, m_generation);
  m_configured = true;
  return true;
}

// ---------------------------------------------------------------------------
// Passive mode
// ---------------------------------------------------------------------------

/**
 * @brief Opens the serial port at the requested baud rate and initialises the parser.
 * @details The assert enforces the precondition that a specific generation was supplied
 *          to the constructor.  A compile-time alternative is available — see the
 *          UbxGNSS.hpp class-level note — but requires making the generation a template
 *          parameter, which changes the public API.  m_configureResult is zero-initialised
 *          so that ubxConfigResult() returns a defined (though empty) struct.
 */
void UbxGNSS::beginPassive(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                            uint32_t baud)
{
  // UNKNOWN is not valid here: the library has no way to determine which
  // message set to expect without running the full configurator.
  // Pass GpsProvider::UBX_M6_MINUS, UBX_M7_M8, or UBX_M9_PLUS to the
  // UbxGNSS constructor before calling beginPassive().
  assert((m_generation != GpsProvider::UNKNOWN) &&
         "UbxGNSS::beginPassive() requires a specific GpsProvider — "
         "pass UBX_M6_MINUS, UBX_M7_M8, or UBX_M9_PLUS to the constructor");

  m_configureResult = UbxConfigResult{};
  serial.end();
  serial.begin(baud, SERIAL_8N1, rxPin, txPin);
  m_parser.begin(serial, m_generation);
  m_configured = true;
}

// ---------------------------------------------------------------------------
// Main loop delegation
// ---------------------------------------------------------------------------

/**
 * @brief Delegates to UbxParser::update() when the parser is active.
 * @details The m_configured guard avoids feeding bytes to the parser before
 *          begin() or beginPassive() has been called.
 */
void UbxGNSS::update()
{
  if (m_configured) {
    m_parser.update();
  }
}

/**
 * @brief Returns true once per new solution; guards against unconfigured state.
 * @return true if a new navigation solution has arrived since the last call.
 */
bool UbxGNSS::hasNewData()
{
  return m_configured && m_parser.hasNewData();
}

/**
 * @brief Returns the fix-validity flag; guards against unconfigured state.
 * @return true if the most recent solution reports a valid 3-D GNSS fix.
 */
bool UbxGNSS::isFixValid() const
{
  return m_configured && m_parser.isFixValid();
}

/**
 * @brief Copies the most recently decoded navigation payload into dest.
 * @details Delegates unconditionally to the parser; dest content is undefined
 *          until at least one complete solution has been assembled.
 */
void UbxGNSS::getPayload(CrsfGpsPayload& dest) const
{
  m_parser.getPayload(dest);
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

/**
 * @brief Returns the m_configured flag.
 * @return true after a successful begin() or any beginPassive() call.
 */
bool UbxGNSS::isConfigured() const
{
  return m_configured;
}

/**
 * @brief Returns the hardware generation in use by the parser.
 * @details When UNKNOWN was passed to the constructor and begin() succeeded,
 *          this reflects the generation detected via MON-VER, not UNKNOWN.
 */
GpsProvider UbxGNSS::getDetectedProvider() const
{
  return m_generation;
}

/**
 * @brief Returns a const reference to the stored configuration result.
 * @details The result is zero-initialised after beginPassive() and after any
 *          failed begin() — check result.status before inspecting other fields.
 */
const UbxConfigResult& UbxGNSS::ubxConfigResult() const
{
  return m_configureResult;
}
