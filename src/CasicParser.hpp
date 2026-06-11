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
#include "GnssParserBase.hpp"
#include "CasicConstants.hpp"

// ---------------------------------------------------------------------------
// CasicParser
//
// Parses a CASIC binary stream (NAV-PV messages) into CrsfGpsPayload fields.
//
// Frame layout:
//   0xBA  0xCE  [lenL lenH]  [class]  [id]  [payload…]  [ck0 ck1 ck2 ck3]
//
// Checksum (U32 LE):
//   Seed   : ck = (id << 24) | (class << 16) | payloadLen
//   Payload: for each complete 4-byte LE word: ck += word
//
// One message per epoch (NAV-PV), no epoch assembly required.
//
// Linkage: this translation unit and GnssParserBase.cpp are the only files
// pulled in when CasicGNSS is used.
// ---------------------------------------------------------------------------

class CasicParser : public GnssParserBase {
public:

  /**
   * @brief Constructor.  Initialises all frame parser state to safe defaults.
   * @details All members are set to zero / SYNC1; begin() must be called
   *          before the parser will produce any output.
   */
  CasicParser();

  /**
   * @brief Binds the parser to a serial port and resets all state.
   * @details The port must already be open at the correct baud rate.  Safe to
   *          call more than once; re-initialises all state on each call.
   * @param serial Reference to an already-open HardwareSerial port.
   */
  void begin(HardwareSerial& serial);

  /**
   * @brief Reads available bytes and advances the CASIC frame parser state machine.
   * @details Stops when no bytes are available or the per-call time budget
   *          (GPS_UPDATE_BUDGET_US) is exhausted.  Does nothing if begin() has
   *          not yet been called.  Call every iteration of loop().
   */
  void update();

private:

  // -------------------------------------------------------------------------
  // CASIC frame parser state machine
  // -------------------------------------------------------------------------

  /**
   * @brief State values for the byte-level CASIC frame parser.
   * @details The CASIC frame header places length before class and ID, so
   *          LEN_L / LEN_H are processed before CLASS / ID — the opposite
   *          order from UBX.  Four checksum states (CK_0 … CK_3) collect
   *          the little-endian 32-bit checksum word.
   */
  enum class CasicState : uint8_t {
    SYNC1, SYNC2, LEN_L, LEN_H, CLASS, ID, PAYLOAD,
    CK_0, CK_1, CK_2, CK_3
  };

  /**
   * @brief Resets all frame parser state to power-on defaults.
   * @details Also calls resetCommonState() to clear the shared output payload and flags.
   */
  void resetState();

  /**
   * @brief Feeds one byte into the CASIC frame state machine.
   * @details Advances through SYNC1 → SYNC2 → LEN_L → LEN_H → CLASS → ID →
   *          PAYLOAD → CK_0 → CK_1 → CK_2 → CK_3.  The 32-bit checksum is
   *          accumulated word-by-word during the PAYLOAD state and compared
   *          against the received bytes in CK_3.  onFrame() is called on a
   *          successful match; any framing or checksum error resets to SYNC1.
   * @param b The next byte from the serial stream.
   */
  void feedByte(uint8_t b);

  /**
   * @brief Dispatches a verified CASIC frame to the appropriate payload processor.
   * @details A static_assert verifies at compile time that CASIC_NAVPV_PAYLOAD_LEN
   *          is a multiple of 4, which is required for the word-based checksum
   *          accumulation.  Only NAV-PV frames of the correct class, ID, and
   *          declared length are processed; all others are silently discarded.
   */
  void onFrame();

  /**
   * @brief Processes a NAV-PV payload and populates all CrsfGpsPayload fields.
   * @details A valid 3-D fix is indicated by posValid >= CASIC_POSVALID_MIN_3D.
   *          Latitude and longitude are read as R8 (double, degrees) and scaled
   *          to 1e-7 integer units for CRSF.  MSL altitude is derived as
   *          (height - sepGeoid); see the sign-convention note in the
   *          implementation for firmware variants with opposite geoid sign.
   *          Ground speed is converted from m/s to hundredths of km/h (× 360).
   *          Heading is normalised to [0°, 360°) before scaling to hundredths.
   */
  void processNavPv();

  // -------------------------------------------------------------------------
  // CASIC frame state
  // -------------------------------------------------------------------------
  CasicState m_state;                    ///< Current state of the byte-level frame parser.
  uint8_t    m_class;                    ///< Message class byte of the frame under assembly.
  uint8_t    m_id;                       ///< Message ID byte of the frame under assembly.
  uint16_t   m_payloadLen;               ///< Payload length declared in the current frame header.
  uint16_t   m_payloadIdx;               ///< Number of payload bytes received so far.
  uint8_t    m_buf[GPS_MAX_PAYLOAD_LEN]; ///< Raw payload accumulation buffer.
  uint32_t   m_ckAccum;                  ///< Running 32-bit checksum accumulator.
  uint8_t    m_ckBuf[4U];               ///< Four received checksum bytes, collected in CK_0–CK_3.
};
