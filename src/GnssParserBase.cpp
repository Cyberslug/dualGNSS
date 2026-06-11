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

#include "GnssParserBase.hpp"
#include <string.h>  // memcpy, memset

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

/**
 * @brief Constructor.  Initialises all shared output state to safe defaults.
 * @details m_serial is left as nullptr until a concrete subclass calls begin();
 *          update() is a no-op until m_serial is bound.
 */
GnssParserBase::GnssParserBase()
  : m_serial(nullptr)
  , m_payload{}
  , m_newData(false)
  , m_fixValid(false)
{
}

// ---------------------------------------------------------------------------
// Public read interface
// ---------------------------------------------------------------------------

/**
 * @brief Returns true exactly once per newly assembled navigation solution.
 * @details Clears the internal flag on read; returns false on all subsequent
 *          calls until the next complete epoch is assembled.
 * @return true if a new solution has arrived since the last call, false otherwise.
 */
bool GnssParserBase::hasNewData()
{
  if (m_newData) {
    m_newData = false;
    return true;
  }
  return false;
}

/**
 * @brief Returns the fix-validity flag for the most recently decoded solution.
 * @return true if the last decoded fix was a valid 3-D GNSS fix.
 */
bool GnssParserBase::isFixValid() const
{
  return m_fixValid;
}

/**
 * @brief Copies the most recently decoded navigation solution into dest.
 * @param dest Reference to a CrsfGpsPayload struct that receives the current solution.
 */
void GnssParserBase::getPayload(CrsfGpsPayload& dest) const
{
  dest = m_payload;
}

// ---------------------------------------------------------------------------
// Protected helpers
// ---------------------------------------------------------------------------

/**
 * @brief Resets the shared output state to power-on defaults.
 * @details Zeroes m_payload via memset and clears both boolean flags.
 *          Call from the concrete subclass resetState() on every reinitialisation.
 */
void GnssParserBase::resetCommonState()
{
  m_newData  = false;
  m_fixValid = false;
  memset(&m_payload, 0, sizeof(m_payload));
}

// ---------------------------------------------------------------------------
// Typed little-endian field accessors
//
// All five functions use memcpy to copy raw bytes into a typed variable.
// This is the only strictly-conforming way to perform type-punning in C++ 
// without invoking aliasing undefined behaviour.(yes punning is what it is
// called, who knew - interpreting data in a type other than that it was declared) 
// The CPU is assumed to be little-endian (true of all ESP32 Xtensa and RISC-V targets).
// ---------------------------------------------------------------------------

/**
 * @brief Reads a uint8_t field from a raw byte buffer at the given offset.
 * @param buf    Pointer to the raw receive buffer.
 * @param offset Byte offset of the field within buf.
 * @param out    Reference that receives the extracted value.
 */
void GnssParserBase::readU1(const uint8_t* buf, uint8_t offset, uint8_t& out)
{
  memcpy(&out, &buf[offset], sizeof(out));
}

/**
 * @brief Reads a uint32_t field from a raw little-endian byte buffer.
 * @param buf    Pointer to the raw receive buffer.
 * @param offset Byte offset of the field within buf.
 * @param out    Reference that receives the extracted value.
 */
void GnssParserBase::readU4(const uint8_t* buf, uint8_t offset, uint32_t& out)
{
  memcpy(&out, &buf[offset], sizeof(out));
}

/**
 * @brief Reads an int32_t field from a raw little-endian byte buffer.
 * @param buf    Pointer to the raw receive buffer.
 * @param offset Byte offset of the field within buf.
 * @param out    Reference that receives the extracted value.
 */
void GnssParserBase::readI4(const uint8_t* buf, uint8_t offset, int32_t& out)
{
  memcpy(&out, &buf[offset], sizeof(out));
}

/**
 * @brief Reads a 32-bit IEEE 754 float field from a raw little-endian byte buffer.
 * @param buf    Pointer to the raw receive buffer.
 * @param offset Byte offset of the field within buf.
 * @param out    Reference that receives the extracted value.
 */
void GnssParserBase::readR4(const uint8_t* buf, uint8_t offset, float& out)
{
  memcpy(&out, &buf[offset], sizeof(out));
}

/**
 * @brief Reads a 64-bit double field from a raw little-endian byte buffer.
 * @details A static_assert verifies sizeof(double) == 8 at compile time.  This
 *          check will fail on AVR-family targets where double is 32 bits; it is
 *          satisfied by all ESP32 Xtensa and RISC-V targets.
 * @param buf    Pointer to the raw receive buffer.
 * @param offset Byte offset of the field within buf.
 * @param out    Reference that receives the extracted value.
 */
void GnssParserBase::readR8(const uint8_t* buf, uint8_t offset, double& out)
{
  // Copying 8 bytes into a double is only correct when double is 64-bit
  // This holds on all ESP32 targets (Xtensa and RISC-V).
  // A port to an AVR-style platform where double == float would silently
  // overwrite 4 bytes beyond the variable; the assert catches that.
  static_assert(sizeof(double) == 8U,
    "readR8 requires 64-bit double — not satisfied on this target");
  memcpy(&out, &buf[offset], sizeof(out));
}

// ---------------------------------------------------------------------------
// Arithmetic helpers
// ---------------------------------------------------------------------------

/**
 * @brief Clamps an already-offset altitude value to the uint16 range [0, 65535].
 * @details The caller must add CRSF_ALT_OFFSET_M before calling.  Negative
 *          values are clamped to zero; values above 65535 are clamped to 65535.
 * @param altM  Altitude in metres after adding the CRSF +1000 m offset.
 * @return Clamped altitude as uint16_t.
 */
uint16_t GnssParserBase::clampAltU16(int32_t altM)
{
  if (altM < 0) {
    return 0U;
  }
  if (altM > 65535) {
    return 65535U;
  }
  return static_cast<uint16_t>(altM);
}

/**
 * @brief Normalises a heading in 1e-5 degrees to hundredths of degrees in [0, 36000).
 * @details One additive or subtractive correction brings any value in the range
 *          (-36,000,000, +36,000,000) into [0, 36,000,000).  The upper-bound branch
 *          handles the edge case of exactly 360° mapping to 0°.  The result is
 *          then divided by 1000 to convert from 1e-5 degrees to hundredths of degrees.
 * @param heading1e5  Heading in 1e-5 degree units, signed.
 * @return Heading in hundredths of degrees, range [0, 35999].
 */
uint16_t GnssParserBase::normaliseHeading1e5(int32_t heading1e5)
{
  int32_t h = heading1e5;

  // Bring h into [0, 36 000 000).  Real GPS heading fields are always
  // within (-36 000 000, +36 000 000), so one correction per branch
  // suffices.  The upper-bound branch handles the edge case of exactly
  // 360.00000° (h == 36 000 000), mapping it to 0° as expected.
  if (h < 0) {
    h += static_cast<int32_t>(36000000);
  } else if (h >= static_cast<int32_t>(36000000)) {
    h -= static_cast<int32_t>(36000000);
  }

  return static_cast<uint16_t>(static_cast<uint32_t>(h) / 1000U);
}
