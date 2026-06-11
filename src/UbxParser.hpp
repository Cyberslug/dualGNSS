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

#pragma once

#include <Arduino.h>
#include "GnssParserBase.hpp"
#include "GpsProvider.hpp"
#include "UbxConstants.hpp"

// ---------------------------------------------------------------------------
// UbxParser
//
// Parses a UBX binary stream into CrsfGpsPayload fields.  Two sub-protocols
// are supported, selected at begin()-time via the generation argument:
//
//   UBX_M7_M8 or UBX_M9_PLUS  — NAV-PVT (single message per epoch)
//   UBX_M6_MINUS               — NAV-POSLLH + NAV-SOL + NAV-VELNED
//                                (epoch assembled from three messages)
//
// In full-mode operation (UbxGNSS::begin) the generation is supplied by
// UbxConfigurator after reading MON-VER, or taken directly from the value
// passed to the UbxGNSS constructor if it was not UNKNOWN.  In passive mode
// (UbxGNSS::beginPassive) it comes from the UbxGNSS constructor and must
// not be UNKNOWN — UbxGNSS enforces this with an assertion before calling
// begin() here.
//
// Linkage: this translation unit and GnssParserBase.cpp are the only files
// pulled in when UbxGNSS is used.
// ---------------------------------------------------------------------------

class UbxParser : public GnssParserBase {
public:

  /**
   * @brief Constructor.  Initialises all frame parser state to safe defaults.
   * @details m_generation is set to UNKNOWN; begin() must be called before
   *          update() will produce any output.
   */
  UbxParser();

  /**
   * @brief Binds the parser to a serial port and selects the UBX sub-protocol.
   * @details The port must already be open at the correct baud rate.  Safe to
   *          call more than once; re-initialises all state on each call.
   *          generation must be UBX_M6_MINUS, UBX_M7_M8, or UBX_M9_PLUS —
   *          UNKNOWN is not valid here; UbxGNSS guarantees it is never passed.
   * @param serial     Reference to an already-open HardwareSerial port.
   * @param generation Hardware generation of the connected module.
   */
  void begin(HardwareSerial& serial, GpsProvider generation);

  /**
   * @brief Reads available bytes and advances the UBX frame parser state machine.
   * @details Stops when no bytes are available or the per-call time budget
   *          (GPS_UPDATE_BUDGET_US) is exhausted.  Does nothing if begin() has
   *          not yet been called.  Call every iteration of loop().
   */
  void update();

private:

  // -------------------------------------------------------------------------
  // UBX frame parser state machine
  // -------------------------------------------------------------------------

  /**
   * @brief State values for the byte-level UBX frame parser.
   */
  enum class UbxState : uint8_t {
    SYNC1, SYNC2, CLASS, ID, LEN_L, LEN_H, PAYLOAD, CK_A, CK_B
  };

  /**
   * @brief Resets all frame parser state and M6- epoch assembly state to defaults.
   * @details Also calls resetCommonState() to clear the shared output payload.
   */
  void resetState();

  /**
   * @brief Feeds one byte into the UBX frame state machine.
   * @details Advances m_state on each call, accumulating the checksum and
   *          buffering the payload.  Calls onFrame() when a complete,
   *          checksum-verified frame has been assembled.
   * @param b The next byte from the serial stream.
   */
  void feedByte(uint8_t b);

  /**
   * @brief Dispatches a verified UBX frame to the appropriate payload processor.
   * @details Only NAV-class frames are processed; all others are silently discarded.
   *          M7+ frames are routed to processM7Pvt(); M6- frames are routed to
   *          processM6Posllh(), processM6Sol(), or processM6Velned() by message ID.
   */
  void onFrame();

  /**
   * @brief Processes a NAV-PVT payload for M7 / M8 / M9 / M10 modules.
   * @details Extracts fix type, flags, satellite count, position, altitude,
   *          ground speed, and heading; populates m_payload and sets m_newData.
   */
  void processM7Pvt();

  /**
   * @brief Processes a NAV-POSLLH payload for M6- modules.
   * @details Extracts the iTOW, longitude, latitude, and height above MSL into
   *          the M6- assembly buffers and sets M6_FLAG_POSLLH.  Calls
   *          assembleM6Solution() if all three epoch messages have arrived.
   */
  void processM6Posllh();

  /**
   * @brief Processes a NAV-SOL payload for M6- modules.
   * @details Extracts fix type, flags, and satellite count.  May arrive in any
   *          order relative to POSLLH and VELNED within the same epoch.  Sets
   *          M6_FLAG_SOL and calls assembleM6Solution() if the epoch is complete.
   */
  void processM6Sol();

  /**
   * @brief Processes a NAV-VELNED payload for M6- modules.
   * @details Extracts 2-D ground speed and vehicle heading into the M6- assembly
   *          buffers and sets M6_FLAG_VELNED.  Calls assembleM6Solution() if all
   *          three epoch messages have arrived.
   */
  void processM6Velned();

  /**
   * @brief Assembles a complete M6- navigation solution from the three epoch buffers.
   * @details Called when M6_FLAGS_ALL is set, indicating POSLLH, SOL, and VELNED have
   *          all arrived with the same iTOW.  Converts raw field values to CRSF units,
   *          populates m_payload, and sets m_newData.  Resets m_m6Flags on exit.
   */
  void assembleM6Solution();

  /**
   * @brief Advances the M6- epoch gate, clearing assembly state when a new iTOW is seen.
   * @details Called at the start of each M6- message processor.  If itow differs from
   *          m_m6AssembleItow the previous (incomplete) epoch is discarded and assembly
   *          restarts from the current message.
   * @param itow GPS time-of-week in milliseconds from the incoming message header.
   */
  void advanceM6Epoch(uint32_t itow);

  // -------------------------------------------------------------------------
  // UBX frame state
  // -------------------------------------------------------------------------
  GpsProvider m_generation;              ///< Active sub-protocol: M6_MINUS, M7_M8, or M9_PLUS.
  UbxState    m_state;                   ///< Current state of the byte-level frame parser.
  uint8_t     m_class;                   ///< Message class byte of the frame under assembly.
  uint8_t     m_id;                      ///< Message ID byte of the frame under assembly.
  uint16_t    m_payloadLen;              ///< Payload length declared in the current frame header.
  uint16_t    m_payloadIdx;              ///< Number of payload bytes received so far.
  uint8_t     m_buf[GPS_MAX_PAYLOAD_LEN];///< Raw payload accumulation buffer.
  uint8_t     m_ckA;                     ///< Fletcher-8 running checksum accumulator A.
  uint8_t     m_ckB;                     ///< Fletcher-8 running checksum accumulator B.

  // -------------------------------------------------------------------------
  // M6- epoch assembly state
  //
  // A complete solution is emitted when POSLLH, SOL, and VELNED have all
  // been received with the same iTOW value (m_m6AssembleItow).
  // m_m6Flags tracks which of the three messages have arrived.
  // -------------------------------------------------------------------------
  uint8_t  m_m6Flags;         ///< Bitmask of M6- epoch messages received (M6_FLAG_*).
  uint32_t m_m6AssembleItow;  ///< iTOW of the epoch currently being assembled.
  int32_t  m_m6Lon;           ///< Longitude from NAV-POSLLH, 1e-7 °.
  int32_t  m_m6Lat;           ///< Latitude from NAV-POSLLH, 1e-7 °.
  int32_t  m_m6Hmsl;          ///< Height above MSL from NAV-POSLLH, mm.
  uint32_t m_m6GSpeed;        ///< 2-D ground speed from NAV-VELNED, cm/s.
  int32_t  m_m6Heading;       ///< Vehicle heading from NAV-VELNED, 1e-5 °, signed.
  uint8_t  m_m6NumSv;         ///< Satellite count from NAV-SOL.
  bool     m_m6FixValid;      ///< Fix validity flag from NAV-SOL.
};
