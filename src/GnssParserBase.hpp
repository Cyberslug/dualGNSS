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

#include <Arduino.h>
#include "CommonStructures.hpp"
#include "CommonParserConstants.hpp"

// ---------------------------------------------------------------------------
// GnssParserBase
//
// Non-polymorphic base shared by UbxParser and CasicParser.
//
// Responsibilities:
//   · Owns the shared output state (m_serial, m_payload, m_newData,
//     m_fixValid).
//   · Exposes the public three-method read interface (hasNewData,
//     isFixValid, getPayload).
//   · Provides protected typed little-endian field accessors and small
//     arithmetic helpers used by both concrete parsers.
//
// No virtual functions are declared here.  Each
// concrete subclass provides its own begin() and update() methods; the
// top-level GNSS classes (UbxGNSS, CasicGNSS) hold the concrete parser
// by value and call through it directly, so no polymorphic dispatch is
// needed.
//
// Not meant for direct instantiation — constructor is protected.
//
// Linking note: GnssParserBase.cpp is always linked when either parser is
// used; it contains no protocol-specific code.
// ---------------------------------------------------------------------------

class GnssParserBase {
public:

  /**
   * @brief Returns true exactly once per newly assembled navigation solution.
   * @details Resets the internal new-data flag on read, so successive calls
   *          return false until the next complete epoch is assembled by the
   *          concrete parser.
   * @return true if a new solution has arrived since the last call, false otherwise.
   */
  bool hasNewData();

  /**
   * @brief Returns true when the most recently decoded solution has a valid 3-D fix.
   * @details Does not reset on read; returns the same value until the next
   *          solution is assembled.  Always false before the first solution arrives.
   * @return true if the last decoded fix was a valid 3-D GNSS fix.
   */
  bool isFixValid() const;

  /**
   * @brief Copies the most recently decoded navigation solution into dest.
   * @details Safe to call at any time, including before a valid fix is available.
   *          Does not reset the new-data flag.  The contents of dest are
   *          undefined until at least one solution has been assembled.
   * @param dest Reference to a CrsfGpsPayload struct that receives the current solution.
   */
  void getPayload(CrsfGpsPayload& dest) const;

protected:

  /**
   * @brief Protected constructor.  Initialises all shared output state to safe defaults.
   * @details Not intended for direct instantiation; call from a derived class constructor.
   */
  GnssParserBase();

  /**
   * @brief Resets the shared output state to its power-on defaults.
   * @details Clears m_payload to zero, sets m_newData and m_fixValid to false.
   *          Must be called from the concrete subclass resetState() whenever
   *          the parser is reinitialised (e.g. from begin()).
   */
  void resetCommonState();

  // -------------------------------------------------------------------------
  // Typed little-endian field accessors
  //
  // Each function copies sizeof(out) bytes from &buf[offset] into out using
  // memcpy, which is the only strictly-conforming way to read a typed value
  // from a raw byte buffer without aliasing undefined behaviour.
  //
  // Correctness preconditions (both satisfied on all ESP32 targets):
  //   · The buffer contains the field in little-endian byte order.
  //   · The CPU is little-endian (ESP32 Xtensa and RISC-V are both LE).
  //
  // readR8 additionally requires sizeof(double) == 8 (64-bit).
  // A static_assert inside the function enforces this at compile time.
  // -------------------------------------------------------------------------

  /**
   * @brief Reads a uint8_t field from a raw byte buffer.
   * @param buf    Pointer to the raw receive buffer.
   * @param offset Byte offset of the field within buf.
   * @param out    Reference that receives the extracted value.
   */
  static void readU1(const uint8_t* buf, uint8_t offset, uint8_t&  out);

  /**
   * @brief Reads a uint32_t field from a raw little-endian byte buffer.
   * @param buf    Pointer to the raw receive buffer.
   * @param offset Byte offset of the field within buf.
   * @param out    Reference that receives the extracted value.
   */
  static void readU4(const uint8_t* buf, uint8_t offset, uint32_t& out);

  /**
   * @brief Reads an int32_t field from a raw little-endian byte buffer.
   * @param buf    Pointer to the raw receive buffer.
   * @param offset Byte offset of the field within buf.
   * @param out    Reference that receives the extracted value.
   */
  static void readI4(const uint8_t* buf, uint8_t offset, int32_t&  out);

  /**
   * @brief Reads a 32-bit float field from a raw little-endian byte buffer.
   * @param buf    Pointer to the raw receive buffer.
   * @param offset Byte offset of the field within buf.
   * @param out    Reference that receives the extracted value.
   */
  static void readR4(const uint8_t* buf, uint8_t offset, float&    out);

  /**
   * @brief Reads a 64-bit double field from a raw little-endian byte buffer.
   * @details Requires sizeof(double) == 8.  A static_assert inside the function
   *          enforces this at compile time; it will fail on AVR targets where
   *          double is 32 bits.
   * @param buf    Pointer to the raw receive buffer.
   * @param offset Byte offset of the field within buf.
   * @param out    Reference that receives the extracted value.
   */
  static void readR8(const uint8_t* buf, uint8_t offset, double&   out);

  // -------------------------------------------------------------------------
  // Arithmetic helpers
  // -------------------------------------------------------------------------

  /**
   * @brief Clamps an already-offset altitude value to the uint16 range [0, 65535].
   * @details The caller must add CRSF_ALT_OFFSET_M to the raw MSL metres value
   *          before calling.  Negative values (depths below the CRSF range) are
   *          clamped to zero; values above 65535 are clamped to 65535.
   * @param altM  Altitude in metres after adding the CRSF +1000 m offset.
   * @return Clamped altitude as uint16_t, saturating at 0 and 65535.
   */
  static uint16_t clampAltU16(int32_t altM);

  /**
   * @brief Normalises a heading in 1e-5 degrees to hundredths of degrees in [0, 36000).
   * @details Maps the half-open range [0°, 360°) to [0, 35999] in hundredths of a
   *          degree.  The edge case of exactly 360° (heading1e5 == 36,000,000) maps
   *          to 0°.  Valid input range: (-36,000,000, +36,000,000); real GPS heading
   *          fields are always within this range.
   * @param heading1e5  Heading in 1e-5 degree units, signed.
   * @return Heading in hundredths of degrees, in the range [0, 35999].
   */
  static uint16_t normaliseHeading1e5(int32_t heading1e5);

  // -------------------------------------------------------------------------
  // Shared output state — written by subclasses, read via public interface.
  // -------------------------------------------------------------------------
  HardwareSerial* m_serial;   ///< Bound serial port; nullptr until begin() is called.
  CrsfGpsPayload  m_payload;  ///< Most recently decoded navigation solution.
  bool            m_newData;  ///< Set true by the subclass when a solution is complete.
  bool            m_fixValid; ///< Fix validity flag for the most recent solution.
};
