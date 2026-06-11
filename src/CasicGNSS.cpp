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

#include "CasicGNSS.hpp"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

/**
 * @brief Constructor.  Initialises all members to safe defaults.
 * @details m_configured is false until begin() or beginPassive() succeeds;
 *          m_configureResult is zero-initialised.
 */
CasicGNSS::CasicGNSS()
  : m_parser()
  , m_configureResult{}
  , m_configured(false)
{
}

// ---------------------------------------------------------------------------
// Full mode
// ---------------------------------------------------------------------------

/**
 * @brief Runs CasicConfigurator::configure(), then initialises the parser on success.
 * @details Resets m_configured and m_configureResult before each attempt so a failed
 *          call leaves a clean diagnostic state.  The configurator leaves the port open
 *          at TARGET_BAUD_RATE on success, so the parser can begin immediately without
 *          reopening the port.
 */
bool CasicGNSS::begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
{
  m_configured      = false;
  m_configureResult = CasicConfigResult{};

  CasicConfigurator cfg;
  m_configureResult = cfg.configure(serial, rxPin, txPin);

  if (m_configureResult.status != CasicConfigStatus::OK) {
    return false;
  }

  // The configurator leaves the port open at TARGET_BAUD_RATE on success,
  // so the parser can start immediately.
  m_parser.begin(serial);
  m_configured = true;
  return true;
}

// ---------------------------------------------------------------------------
// Passive mode
// ---------------------------------------------------------------------------

/**
 * @brief Opens the serial port at the requested baud rate and initialises the parser.
 * @details No generation check is required because the CASIC protocol stack has 
 *          a single protocol definition, all known CASIC modules have NAV-PV.  
 *          m_configureResult is zero-initialised so that casicConfigResult() 
 *          returns a defined (though empty) struct.
 */
void CasicGNSS::beginPassive(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                              uint32_t baud)
{
  m_configureResult = CasicConfigResult{};
  serial.end();
  serial.begin(baud, SERIAL_8N1, rxPin, txPin);
  m_parser.begin(serial);
  m_configured = true;
}

// ---------------------------------------------------------------------------
// Main loop delegation
// ---------------------------------------------------------------------------

/**
 * @brief Delegates to CasicParser::update() when the parser is active.
 * @details The m_configured guard avoids feeding bytes to the parser before
 *          begin() or beginPassive() has been called.
 */
void CasicGNSS::update()
{
  if (m_configured) {
    m_parser.update();
  }
}

/**
 * @brief Returns true once per new solution; guards against unconfigured state.
 * @return true if a new navigation solution has arrived since the last call.
 */
bool CasicGNSS::hasNewData()
{
  return m_configured && m_parser.hasNewData();
}

/**
 * @brief Returns the fix-validity flag; guards against unconfigured state.
 * @return true if the most recent solution reports a valid 3-D GNSS fix.
 */
bool CasicGNSS::isFixValid() const
{
  return m_configured && m_parser.isFixValid();
}

/**
 * @brief Copies the most recently decoded navigation payload into dest.
 * @details Delegates unconditionally to the parser; dest content is undefined
 *          until at least one complete solution has been assembled.
 *          *TODO* is the typed payload too restrictive? Examine.
 */
void CasicGNSS::getPayload(CrsfGpsPayload& dest) const
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
bool CasicGNSS::isConfigured() const
{
  return m_configured;
}

/**
 * @brief Returns a const reference to the stored configuration result.
 * @details The result is zero-initialised after beginPassive() and after any
 *          failed begin() — check result.status before inspecting other fields.
 */
const CasicConfigResult& CasicGNSS::casicConfigResult() const
{
  return m_configureResult;
}
