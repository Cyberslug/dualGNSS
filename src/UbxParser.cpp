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

#include "UbxParser.hpp"
#include <string.h>  // memcpy, memset

// ---------------------------------------------------------------------------
// Construction / initialisation
// ---------------------------------------------------------------------------

/**
 * @brief Constructor.  Initialises all frame parser and epoch assembly state.
 * @details All members are set to zero / false / UNKNOWN; begin() must be
 *          called before the parser will produce any output.
 */
UbxParser::UbxParser()
  : GnssParserBase()
  , m_generation(GpsProvider::UNKNOWN)
  , m_state(UbxState::SYNC1)
  , m_class(0U)
  , m_id(0U)
  , m_payloadLen(0U)
  , m_payloadIdx(0U)
  , m_buf{}
  , m_ckA(0U)
  , m_ckB(0U)
  , m_m6Flags(0U)
  , m_m6AssembleItow(0U)
  , m_m6Lon(0)
  , m_m6Lat(0)
  , m_m6Hmsl(0)
  , m_m6GSpeed(0U)
  , m_m6Heading(0)
  , m_m6NumSv(0U)
  , m_m6FixValid(false)
{
}

/**
 * @brief Binds the parser to a serial port and selects the UBX sub-protocol.
 * @details Re-initialises all state so the function is safe to call more than once.
 * @param serial     Reference to an already-open HardwareSerial port.
 * @param generation Hardware generation of the connected module (not UNKNOWN).
 */
void UbxParser::begin(HardwareSerial& serial, GpsProvider generation)
{
  m_serial     = &serial;
  m_generation = generation;
  resetState();
}

/**
 * @brief Resets all frame parser and M6- epoch assembly state to power-on defaults.
 * @details Also calls resetCommonState() to clear the shared output payload and flags.
 */
void UbxParser::resetState()
{
  // UBX frame state.
  m_state      = UbxState::SYNC1;
  m_class      = 0U;
  m_id         = 0U;
  m_payloadLen = 0U;
  m_payloadIdx = 0U;
  m_ckA        = 0U;
  m_ckB        = 0U;
  memset(m_buf, 0, sizeof(m_buf));

  // M6- epoch assembly state.
  m_m6Flags        = 0U;
  m_m6AssembleItow = 0U;
  m_m6Lon          = 0;
  m_m6Lat          = 0;
  m_m6Hmsl         = 0;
  m_m6GSpeed       = 0U;
  m_m6Heading      = 0;
  m_m6NumSv        = 0U;
  m_m6FixValid     = false;

  resetCommonState();
}

// ---------------------------------------------------------------------------
// Main loop interface
// ---------------------------------------------------------------------------

/**
 * @brief Reads available bytes and advances the UBX frame parser state machine.
 * @details Loops until no bytes are available or GPS_UPDATE_BUDGET_US has
 *          elapsed.  The unsigned subtraction used for the time check handles
 *          the approximately 70-minute micros() roll-over correctly.
 *          Does nothing if m_serial is nullptr (begin() not yet called).
 */
void UbxParser::update()
{
  if (m_serial == nullptr) {
    return;
  }

  const uint32_t startUs = micros();

  while (m_serial->available() > 0) {
    const uint8_t b = static_cast<uint8_t>(m_serial->read());
    feedByte(b);

    // Unsigned subtraction handles the ~70-minute micros() roll-over.
    if ((micros() - startUs) >= GPS_UPDATE_BUDGET_US) {
      break;
    }
  }
}

// ---------------------------------------------------------------------------
// UBX byte-level frame parser
//
// Frame layout:
//   0xB5  0x62  [class]  [id]  [lenL lenH]  [payload…]  [CK_A CK_B]
//
// Checksum: Fletcher-8 over class, id, lenL, lenH, and all payload bytes.
// CK_A and CK_B are reset when the CLASS byte arrives (start of the
// checksum-covered region) and updated incrementally for every subsequent
// byte up to and including the last payload byte.
// ---------------------------------------------------------------------------

/**
 * @brief Feeds one byte into the UBX frame state machine.
 * @details Advances through SYNC1 → SYNC2 → CLASS → ID → LEN_L → LEN_H →
 *          PAYLOAD → CK_A → CK_B, calling onFrame() on a checksum match.
 *          Any framing error resets the machine to SYNC1.
 * @param b The next byte from the serial stream.
 */
void UbxParser::feedByte(uint8_t b)
{
  switch (m_state) {

    case UbxState::SYNC1:
      if (b == UBX_SYNC1) {
        m_state = UbxState::SYNC2;
      }
      break;

    case UbxState::SYNC2:
      if (b == UBX_SYNC2) {
        m_state = UbxState::CLASS;
      } else {
        // A second 0xB5 may be the start of a real frame; any other byte resets.
        m_state = (b == UBX_SYNC1) ? UbxState::SYNC2 : UbxState::SYNC1;
      }
      break;

    case UbxState::CLASS:
      m_class = b;
      // Initialise Fletcher-8 checksums at the start of the covered region.
      m_ckA   = 0U;
      m_ckB   = 0U;
      m_ckA  += b;       // Include the class byte in the running checksum.
      m_ckB  += m_ckA;
      m_state = UbxState::ID;
      break;

    case UbxState::ID:
      m_id    = b;
      m_ckA  += b;
      m_ckB  += m_ckA;
      m_state = UbxState::LEN_L;
      break;

    case UbxState::LEN_L:
      m_payloadLen = static_cast<uint16_t>(b);
      m_ckA       += b;
      m_ckB       += m_ckA;
      m_state      = UbxState::LEN_H;
      break;

    case UbxState::LEN_H:
      m_payloadLen |= static_cast<uint16_t>(static_cast<uint32_t>(b) << 8U);
      m_ckA        += b;
      m_ckB        += m_ckA;

      if (m_payloadLen > GPS_MAX_PAYLOAD_LEN) {
        // Frame too large for the buffer — discard and resync.
        m_state = UbxState::SYNC1;
      } else if (m_payloadLen == 0U) {
        // Zero-payload frame (e.g. a poll): skip straight to checksum bytes.
        m_state = UbxState::CK_A;
      } else {
        m_payloadIdx = 0U;
        m_state      = UbxState::PAYLOAD;
      }
      break;

    case UbxState::PAYLOAD:
      m_buf[m_payloadIdx] = b;
      m_payloadIdx++;
      m_ckA += b;
      m_ckB += m_ckA;

      if (m_payloadIdx == m_payloadLen) {
        m_state = UbxState::CK_A;
      }
      break;

    case UbxState::CK_A:
      if (b == m_ckA) {
        m_state = UbxState::CK_B;
      } else {
        m_state = UbxState::SYNC1;
      }
      break;

    case UbxState::CK_B:
      if (b == m_ckB) {
        onFrame();
      }
      // Always return to SYNC1 after the final checksum byte, regardless of match.
      m_state = UbxState::SYNC1;
      break;
  }
}

// ---------------------------------------------------------------------------
// Frame dispatch
// ---------------------------------------------------------------------------

/**
 * @brief Dispatches a verified UBX frame to the appropriate payload processor.
 * @details Only UBX_CLASS_NAV frames are processed; all others are discarded.
 *          For M7+ modules only NAV-PVT is accepted.  For M6- modules the
 *          three epoch messages (NAV-POSLLH, NAV-SOL, NAV-VELNED) are each
 *          routed to their dedicated processor by message ID.
 */
void UbxParser::onFrame()
{
  if (m_class != UBX_CLASS_NAV) {
    return;
  }

  if (m_generation != GpsProvider::UBX_M6_MINUS) {
    // M7/M8 and M9/M10 all use NAV-PVT.
    if ((m_id == UBX_ID_NAV_PVT) && (m_payloadLen == UBX_NAVPVT_PAYLOAD_LEN)) {
      processM7Pvt();
    }
  } else {
    // UBX_M6_MINUS: epoch assembled from three message types.
    switch (m_id) {
      case UBX_ID_NAV_POSLLH:
        if (m_payloadLen == UBX_NAVPOSLLH_PAYLOAD_LEN) {
          processM6Posllh();
        }
        break;

      case UBX_ID_NAV_SOL:
        if (m_payloadLen == UBX_NAVSOL_PAYLOAD_LEN) {
          processM6Sol();
        }
        break;

      case UBX_ID_NAV_VELNED:
        if (m_payloadLen == UBX_NAVVELNED_PAYLOAD_LEN) {
          processM6Velned();
        }
        break;

      default:
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// UBX M7+ — NAV-PVT
// ---------------------------------------------------------------------------

/**
 * @brief Processes a NAV-PVT payload for M7 / M8 / M9 / M10 modules.
 * @details A fix is considered valid when the gnssFixOK flag (bit 0 of the
 *          flags byte) is set and fixType equals 3 (3-D fix).  Ground speed
 *          is converted from mm/s to hundredths of km/h (× 36 / 100).
 *          Altitude is converted from mm above MSL to metres, then the CRSF
 *          +1000 m offset is applied.
 */
void UbxParser::processM7Pvt()
{
  uint8_t fixType = 0U;
  uint8_t flags   = 0U;
  uint8_t numSv   = 0U;
  readU1(m_buf, UBX_PVT_OFF_FIXTYPE, fixType);
  readU1(m_buf, UBX_PVT_OFF_FLAGS,   flags);
  readU1(m_buf, UBX_PVT_OFF_NUMSV,   numSv);

  m_fixValid = ((flags & UBX_FLAG_GNSSOK) != 0U) && (fixType == UBX_FIXTYPE_3D);

  int32_t lon     = 0;
  int32_t lat     = 0;
  int32_t hmsl    = 0;
  int32_t gSpeed  = 0;
  int32_t headMot = 0;
  readI4(m_buf, UBX_PVT_OFF_LON,     lon);
  readI4(m_buf, UBX_PVT_OFF_LAT,     lat);
  readI4(m_buf, UBX_PVT_OFF_HMSL,    hmsl);
  readI4(m_buf, UBX_PVT_OFF_GSPEED,  gSpeed);
  readI4(m_buf, UBX_PVT_OFF_HEADMOT, headMot);

  m_payload.latitude  = lat;
  m_payload.longitude = lon;

  // hMSL is in mm; divide by 1000 to get metres, then add CRSF offset.
  m_payload.altitude = clampAltU16(
      (hmsl / static_cast<int32_t>(1000)) + CRSF_ALT_OFFSET_M);

  // gSpeed is I4, mm/s (always >= 0 per spec); mm/s → hundredths of km/h: ×36/100.
  m_payload.groundspeed = static_cast<uint16_t>(
      static_cast<uint32_t>(gSpeed) * 36U / 100U);

  // headMot: I4, 1e-5 °; normalise to [0, 36 000 000) then express in hundredths.
  m_payload.heading = normaliseHeading1e5(headMot);

  m_payload.satellites = numSv;
  m_newData = true;
}

// ---------------------------------------------------------------------------
// UBX M6- — epoch gate
//
// Called at the start of each M6- message processor.  Clears assembly state
// when itow belongs to a new epoch.
// ---------------------------------------------------------------------------

/**
 * @brief Advances the M6- epoch gate, discarding stale assembly state on an iTOW change.
 * @details If itow differs from m_m6AssembleItow the previous (likely incomplete) epoch
 *          is abandoned: m_m6Flags is cleared and m_m6AssembleItow is updated.
 * @param itow GPS time-of-week in milliseconds from the incoming message.
 */
void UbxParser::advanceM6Epoch(uint32_t itow)
{
  if (itow != m_m6AssembleItow) {
    m_m6Flags        = 0U;
    m_m6AssembleItow = itow;
  }
}

// ---------------------------------------------------------------------------
// UBX M6- — NAV-POSLLH
// ---------------------------------------------------------------------------

/**
 * @brief Processes a NAV-POSLLH payload, storing position fields for epoch assembly.
 * @details Sets M6_FLAG_POSLLH and calls assembleM6Solution() if all three epoch
 *          messages have now arrived.
 */
void UbxParser::processM6Posllh()
{
  uint32_t itow = 0U;
  readU4(m_buf, UBX_POSLLH_OFF_ITOW, itow);
  advanceM6Epoch(itow);

  readI4(m_buf, UBX_POSLLH_OFF_LON,  m_m6Lon);
  readI4(m_buf, UBX_POSLLH_OFF_LAT,  m_m6Lat);
  readI4(m_buf, UBX_POSLLH_OFF_HMSL, m_m6Hmsl);
  m_m6Flags |= M6_FLAG_POSLLH;

  if (m_m6Flags == M6_FLAGS_ALL) {
    assembleM6Solution();
  }
}

// ---------------------------------------------------------------------------
// UBX M6- — NAV-SOL
//
// Provides fix validity and satellite count.  May arrive in any order
// relative to POSLLH and VELNED within the same epoch.
// ---------------------------------------------------------------------------

/**
 * @brief Processes a NAV-SOL payload, storing fix validity and satellite count.
 * @details Sets M6_FLAG_SOL and calls assembleM6Solution() if all three epoch
 *          messages have now arrived.
 */
void UbxParser::processM6Sol()
{
  uint32_t itow   = 0U;
  uint8_t  gpsFix = 0U;
  uint8_t  flags  = 0U;
  uint8_t  numSv  = 0U;

  readU4(m_buf, UBX_SOL_OFF_ITOW,       itow);
  advanceM6Epoch(itow);

  readU1(m_buf, UBX_SOL_OFF_GPSFIXTYPE, gpsFix);
  readU1(m_buf, UBX_SOL_OFF_FLAGS,      flags);
  readU1(m_buf, UBX_SOL_OFF_NUMSV,      numSv);

  m_m6FixValid = ((flags & UBX_FLAG_GNSSOK) != 0U) && (gpsFix == UBX_FIXTYPE_3D);
  m_m6NumSv    = numSv;
  m_m6Flags   |= M6_FLAG_SOL;

  if (m_m6Flags == M6_FLAGS_ALL) {
    assembleM6Solution();
  }
}

// ---------------------------------------------------------------------------
// UBX M6- — NAV-VELNED
// ---------------------------------------------------------------------------

/**
 * @brief Processes a NAV-VELNED payload, storing ground speed and heading.
 * @details Sets M6_FLAG_VELNED and calls assembleM6Solution() if all three
 *          epoch messages have now arrived.
 */
void UbxParser::processM6Velned()
{
  uint32_t itow    = 0U;
  uint32_t gSpeed  = 0U;
  int32_t  heading = 0;

  readU4(m_buf, UBX_VELNED_OFF_ITOW,    itow);
  advanceM6Epoch(itow);

  readU4(m_buf, UBX_VELNED_OFF_GSPEED,  gSpeed);
  readI4(m_buf, UBX_VELNED_OFF_HEADING, heading);

  m_m6GSpeed  = gSpeed;
  m_m6Heading = heading;
  m_m6Flags  |= M6_FLAG_VELNED;

  if (m_m6Flags == M6_FLAGS_ALL) {
    assembleM6Solution();
  }
}

// ---------------------------------------------------------------------------
// UBX M6- — epoch assembly
//
// Called when POSLLH, SOL, and VELNED have all arrived for the same iTOW.
// ---------------------------------------------------------------------------

/**
 * @brief Assembles a complete M6- solution from the three buffered epoch messages.
 * @details Converts each field to CRSF units, populates m_payload, and sets
 *          m_newData.  m_m6Flags is cleared on entry so the next epoch can begin
 *          accumulating immediately.  Unit conversions:
 *            altitude  : mm (hMSL) → metres → +1000 m CRSF offset
 *            groundspeed: cm/s → hundredths of km/h  (× 36 / 10)
 *            heading   : 1e-5 ° → hundredths of °   (normaliseHeading1e5)
 */
void UbxParser::assembleM6Solution()
{
  m_m6Flags  = 0U;
  m_fixValid = m_m6FixValid;

  m_payload.latitude  = m_m6Lat;
  m_payload.longitude = m_m6Lon;

  // hMSL in mm → metres → add CRSF offset.
  m_payload.altitude = clampAltU16(
      (m_m6Hmsl / static_cast<int32_t>(1000)) + CRSF_ALT_OFFSET_M);

  // NAV-VELNED gSpeed: U4, cm/s.  cm/s → hundredths of km/h: ×36/10.
  m_payload.groundspeed = static_cast<uint16_t>(m_m6GSpeed * 36U / 10U);

  m_payload.heading    = normaliseHeading1e5(m_m6Heading);
  m_payload.satellites = m_m6NumSv;
  m_newData = true;
}
