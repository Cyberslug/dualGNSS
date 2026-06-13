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
  , m_m6Height(0)
  , m_m6Hmsl(0)
  , m_m6HAcc(0U)
  , m_m6VAcc(0U)
  , m_m6VelN(0)
  , m_m6VelE(0)
  , m_m6VelD(0)
  , m_m6GSpeed(0U)
  , m_m6Heading(0)
  , m_m6SAcc(0U)
  , m_m6CAcc(0U)
  , m_m6NumSv(0U)
  , m_m6FixType(0U)
  , m_m6PDop(0U)
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
  m_m6Height       = 0;
  m_m6Hmsl         = 0;
  m_m6HAcc         = 0U;
  m_m6VAcc         = 0U;
  m_m6VelN         = 0;
  m_m6VelE         = 0;
  m_m6VelD         = 0;
  m_m6GSpeed       = 0U;
  m_m6Heading      = 0;
  m_m6SAcc         = 0U;
  m_m6CAcc         = 0U;
  m_m6NumSv        = 0U;
  m_m6FixType      = 0U;
  m_m6PDop         = 0U;
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

      case UBX_ID_NAV_TIMEUTC:
        if (m_payloadLen == UBX_NAVTIMEUTC_PAYLOAD_LEN) {
          processM6TimeUtc();
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
 * @details Reads the complete set of usable fields from the 92-byte payload.
 *          Fix validity: gnssFixOK && fixType == 3 (3-D).
 *          Time fields: year…millisecond populated from the UTC time group
 *          (offsets 4–16); DATE_VALID and TIME_VALID flags set from the valid
 *          byte (offset 11 bits 0 and 1).  Millisecond derived from nano (I4 ns)
 *          by integer division; clamped to [0, 999].
 *          All other fields stored in natural GnssData units.
 */
void UbxParser::processM7Pvt()
{
  // Fix status
  uint8_t fixType = 0U;
  uint8_t flags   = 0U;
  uint8_t numSv   = 0U;
  readU1(m_buf, UBX_PVT_OFF_FIXTYPE, fixType);
  readU1(m_buf, UBX_PVT_OFF_FLAGS,   flags);
  readU1(m_buf, UBX_PVT_OFF_NUMSV,   numSv);

  const bool fixOk = ((flags & UBX_FLAG_GNSSOK) != 0U) && (fixType == UBX_FIXTYPE_3D);
  m_payload.fixType    = fixType;
  m_payload.validFlags = fixOk
      ? static_cast<uint8_t>(GNSS_FLAG_FIX_OK | GNSS_FLAG_VEL_VALID)
      : 0U;

  // UTC time fields (offsets 4–16)
  uint16_t year  = 0U;
  uint8_t  month = 0U, day = 0U, hour = 0U, min = 0U, sec = 0U, valid = 0U;
  int32_t  nano  = 0;
  readU2(m_buf, UBX_PVT_OFF_YEAR,  year);
  readU1(m_buf, UBX_PVT_OFF_MONTH, month);
  readU1(m_buf, UBX_PVT_OFF_DAY,   day);
  readU1(m_buf, UBX_PVT_OFF_HOUR,  hour);
  readU1(m_buf, UBX_PVT_OFF_MIN,   min);
  readU1(m_buf, UBX_PVT_OFF_SEC,   sec);
  readU1(m_buf, UBX_PVT_OFF_VALID, valid);
  readI4(m_buf, UBX_PVT_OFF_NANO,  nano);

  m_payload.year        = year;
  m_payload.month       = month;
  m_payload.day         = day;
  m_payload.hour        = hour;
  m_payload.minute      = min;
  m_payload.second      = sec;
  // Millisecond: nano is the nanosecond remainder of the UTC second, range ±1e9.
  // Clamp to [0, 999] — negative nano means the reported second hasn't started yet.
  m_payload.millisecond = (nano >= 0)
      ? static_cast<uint16_t>(static_cast<uint32_t>(nano) / 1000000U)
      : 0U;

  // Accumulate date/time validity into validFlags
  if ((valid & 0x01U) != 0U) { m_payload.validFlags |= GNSS_FLAG_DATE_VALID; }
  if ((valid & 0x02U) != 0U) { m_payload.validFlags |= GNSS_FLAG_TIME_VALID; }

  // Position fields
  int32_t lon    = 0;
  int32_t lat    = 0;
  int32_t height = 0;
  int32_t hmsl   = 0;
  readI4(m_buf, UBX_PVT_OFF_LON,    lon);
  readI4(m_buf, UBX_PVT_OFF_LAT,    lat);
  readI4(m_buf, UBX_PVT_OFF_HEIGHT, height);
  readI4(m_buf, UBX_PVT_OFF_HMSL,   hmsl);

  // Position accuracy fields
  uint32_t hAcc = 0U;
  uint32_t vAcc = 0U;
  readU4(m_buf, UBX_PVT_OFF_HACC, hAcc);
  readU4(m_buf, UBX_PVT_OFF_VACC, vAcc);

  // NED velocity fields
  int32_t velN   = 0;
  int32_t velE   = 0;
  int32_t velD   = 0;
  int32_t gSpeed = 0;
  readI4(m_buf, UBX_PVT_OFF_VELN,   velN);
  readI4(m_buf, UBX_PVT_OFF_VELE,   velE);
  readI4(m_buf, UBX_PVT_OFF_VELD,   velD);
  readI4(m_buf, UBX_PVT_OFF_GSPEED, gSpeed);

  // Heading and accuracy fields
  int32_t  headMot = 0;
  uint32_t sAcc    = 0U;
  uint32_t headAcc = 0U;
  uint16_t pDop    = 0U;
  readI4(m_buf, UBX_PVT_OFF_HEADMOT, headMot);
  readU4(m_buf, UBX_PVT_OFF_SACC,    sAcc);
  readU4(m_buf, UBX_PVT_OFF_HEADACC, headAcc);
  readU2(m_buf, UBX_PVT_OFF_PDOP,    pDop);

  m_payload.latitude      = lat;
  m_payload.longitude     = lon;
  m_payload.altMSL        = hmsl;
  m_payload.altEllipsoid  = height;
  m_payload.hAcc          = hAcc;
  m_payload.vAcc          = vAcc;
  m_payload.velN          = velN;
  m_payload.velE          = velE;
  m_payload.velD          = velD;
  m_payload.gSpeed        = gSpeed;
  m_payload.headMot       = normaliseHeadingRaw1e5(headMot);
  m_payload.sAcc          = sAcc;
  m_payload.headAcc       = headAcc;
  m_payload.pDOP          = pDop;
  m_payload.satellites    = numSv;
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
 * @brief Processes a NAV-POSLLH payload, storing position and accuracy fields.
 * @details Sets M6_FLAG_POSLLH and calls assembleM6Solution() if all three epoch
 *          messages have now arrived.
 */
void UbxParser::processM6Posllh()
{
  uint32_t itow = 0U;
  readU4(m_buf, UBX_POSLLH_OFF_ITOW, itow);
  advanceM6Epoch(itow);

  readI4(m_buf, UBX_POSLLH_OFF_LON,    m_m6Lon);
  readI4(m_buf, UBX_POSLLH_OFF_LAT,    m_m6Lat);
  readI4(m_buf, UBX_POSLLH_OFF_HEIGHT, m_m6Height);  // mm above WGS84 ellipsoid
  readI4(m_buf, UBX_POSLLH_OFF_HMSL,   m_m6Hmsl);   // mm above MSL
  readU4(m_buf, UBX_POSLLH_OFF_HACC,   m_m6HAcc);   // mm
  readU4(m_buf, UBX_POSLLH_OFF_VACC,   m_m6VAcc);   // mm
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
 * @brief Processes a NAV-SOL payload, storing fix validity, pDOP, and satellite count.
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
  readU2(m_buf, UBX_SOL_OFF_PDOP,       m_m6PDop);  // U2, dimensionless × 100
  readU1(m_buf, UBX_SOL_OFF_NUMSV,      numSv);

  m_m6FixValid = ((flags & UBX_FLAG_GNSSOK) != 0U) && (gpsFix == UBX_FIXTYPE_3D);
  m_m6FixType  = gpsFix;
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
 * @brief Processes a NAV-VELNED payload, storing NED velocities, speed, heading,
 *        and accuracy fields.
 * @details Sets M6_FLAG_VELNED and calls assembleM6Solution() if all three epoch
 *          messages have now arrived.
 */
void UbxParser::processM6Velned()
{
  uint32_t itow = 0U;
  readU4(m_buf, UBX_VELNED_OFF_ITOW, itow);
  advanceM6Epoch(itow);

  readI4(m_buf, UBX_VELNED_OFF_VELN,    m_m6VelN);    // I4, cm/s north (signed)
  readI4(m_buf, UBX_VELNED_OFF_VELE,    m_m6VelE);    // I4, cm/s east  (signed)
  readI4(m_buf, UBX_VELNED_OFF_VELD,    m_m6VelD);    // I4, cm/s down  (signed)
  readU4(m_buf, UBX_VELNED_OFF_GSPEED,  m_m6GSpeed);  // U4, cm/s 2-D ground speed
  readI4(m_buf, UBX_VELNED_OFF_HEADING, m_m6Heading); // I4, 1e-5 °
  readU4(m_buf, UBX_VELNED_OFF_SACC,    m_m6SAcc);    // U4, cm/s
  readU4(m_buf, UBX_VELNED_OFF_CACC,    m_m6CAcc);    // U4, 1e-5 °
  m_m6Flags |= M6_FLAG_VELNED;

  if (m_m6Flags == M6_FLAGS_ALL) {
    assembleM6Solution();
  }
}

// ---------------------------------------------------------------------------
// UBX M6- — NAV-TIMEUTC
// ---------------------------------------------------------------------------

/**
 * @brief Processes a NAV-TIMEUTC payload, updating UTC time fields in GnssData.
 * @details Decoupled from the epoch gate — called whenever a NAV-TIMEUTC frame
 *          arrives, regardless of whether POSLLH/SOL/VELNED have been received.
 *          Does not set m_newData.  Sets GNSS_FLAG_DATE_VALID and
 *          GNSS_FLAG_TIME_VALID in validFlags when bit 2 (validUTC) of the valid
 *          byte is set; preserves all other validFlags bits.
 */
void UbxParser::processM6TimeUtc()
{
  uint16_t year  = 0U;
  uint8_t  month = 0U, day = 0U, hour = 0U, min = 0U, sec = 0U, valid = 0U;
  int32_t  nano  = 0;

  readU2(m_buf, UBX_TIMEUTC_OFF_YEAR,  year);
  readU1(m_buf, UBX_TIMEUTC_OFF_MONTH, month);
  readU1(m_buf, UBX_TIMEUTC_OFF_DAY,   day);
  readU1(m_buf, UBX_TIMEUTC_OFF_HOUR,  hour);
  readU1(m_buf, UBX_TIMEUTC_OFF_MIN,   min);
  readU1(m_buf, UBX_TIMEUTC_OFF_SEC,   sec);
  readU1(m_buf, UBX_TIMEUTC_OFF_VALID, valid);
  readI4(m_buf, UBX_TIMEUTC_OFF_NANO,  nano);

  m_payload.year        = year;
  m_payload.month       = month;
  m_payload.day         = day;
  m_payload.hour        = hour;
  m_payload.minute      = min;
  m_payload.second      = sec;
  // Millisecond from nanosecond fraction; clamped to zero if nano is negative.
  m_payload.millisecond = (nano >= 0)
      ? static_cast<uint16_t>(static_cast<uint32_t>(nano) / 1000000U)
      : 0U;

  // Update time flags without disturbing fix/velocity flags.
  // validUTC (bit 2) indicates both date and time are valid.
  m_payload.validFlags &= ~(GNSS_FLAG_DATE_VALID | GNSS_FLAG_TIME_VALID);
  if ((valid & 0x04U) != 0U) {
    m_payload.validFlags |= (GNSS_FLAG_DATE_VALID | GNSS_FLAG_TIME_VALID);
  }
}
//
// Called when POSLLH, SOL, and VELNED have all arrived for the same iTOW.
// ---------------------------------------------------------------------------

/**
 * @brief Assembles a complete M6- solution from the three buffered epoch messages.
 * @details Populates m_payload in natural units and sets m_newData.  m_m6Flags
 *          is cleared on entry so the next epoch can begin accumulating
 *          immediately.  Unit conversions applied:
 *            velN / velE / velD / gSpeed : cm/s → mm/s (× 10)
 *            sAcc                        : cm/s → mm/s (× 10)
 *          All other fields are in their canonical GnssData units already.
 *          Protocol-specific conversions for getPayload() are applied in
 *          GnssParserBase::getPayload().
 */
void UbxParser::assembleM6Solution()
{
  m_m6Flags = 0U;
  // Preserve DATE_VALID / TIME_VALID bits set by processM6TimeUtc();
  // FIX_OK and VEL_VALID are overwritten based on this epoch's validity.
  const uint8_t timeFlags = m_payload.validFlags &
      static_cast<uint8_t>(GNSS_FLAG_DATE_VALID | GNSS_FLAG_TIME_VALID);
  m_payload.fixType    = m_m6FixType;
  m_payload.validFlags = static_cast<uint8_t>(timeFlags |
      (m_m6FixValid
          ? static_cast<uint8_t>(GNSS_FLAG_FIX_OK | GNSS_FLAG_VEL_VALID)
          : 0U));

  // Position (direct — already in mm and 1e-7 °)
  m_payload.latitude      = m_m6Lat;
  m_payload.longitude     = m_m6Lon;
  m_payload.altMSL        = m_m6Hmsl;    // mm
  m_payload.altEllipsoid  = m_m6Height;  // mm above WGS84 ellipsoid

  // Position accuracy (direct — already in mm)
  m_payload.hAcc          = m_m6HAcc;
  m_payload.vAcc          = m_m6VAcc;

  // NED velocity: cm/s → mm/s (× 10)
  m_payload.velN          = m_m6VelN * 10;
  m_payload.velE          = m_m6VelE * 10;
  m_payload.velD          = m_m6VelD * 10;

  // 2-D ground speed: cm/s → mm/s (× 10)
  m_payload.gSpeed        = static_cast<int32_t>(m_m6GSpeed * 10U);

  // Heading: 1e-5 °, normalised to [0, 36 000 000)
  m_payload.headMot       = normaliseHeadingRaw1e5(m_m6Heading);

  // Speed accuracy: cm/s → mm/s (× 10)
  m_payload.sAcc          = m_m6SAcc * 10U;

  // Heading accuracy: 1e-5 ° (direct from NAV-VELNED cAcc)
  m_payload.headAcc       = m_m6CAcc;

  // Position DOP: dimensionless × 100 (direct from NAV-SOL)
  m_payload.pDOP          = m_m6PDop;

  m_payload.satellites    = m_m6NumSv;
  m_newData = true;
}
