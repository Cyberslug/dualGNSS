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

#include "CasicParser.hpp"
#include <math.h>   // sqrtf — variance → 1-sigma accuracy conversion
#include <string.h> // memcpy, memset

// ---------------------------------------------------------------------------
// Construction / initialisation
// ---------------------------------------------------------------------------

/**
 * @brief Constructor.  Initialises all frame parser states to safe defaults.
 * @details All members are zero-initialised and the state machine is set to
 *          SYNC1.  begin() must be called before update() will produce output.
 */
CasicParser::CasicParser()
  : GnssParserBase()
  , m_state(CasicState::SYNC1)
  , m_class(0U)
  , m_id(0U)
  , m_payloadLen(0U)
  , m_payloadIdx(0U)
  , m_buf{}
  , m_ckAccum(0U)
  , m_ckBuf{}
{
}

/**
 * @brief Binds the parser to a serial port and resets all state.
 * @details Re-initialises all state so this function is safe to call more than once.
 * @param serial Reference to an already-open HardwareSerial port.
 */
void CasicParser::begin(HardwareSerial& serial)
{
  m_serial = &serial;
  resetState();
}

/**
 * @brief Resets all frame parser state to power-on defaults.
 * @details Clears the payload buffer and checksum buffer via memset, resets
 *          all counters and state, then calls resetCommonState() to clear the
 *          shared output payload and flags.
 */
void CasicParser::resetState()
{
  m_state      = CasicState::SYNC1;
  m_class      = 0U;
  m_id         = 0U;
  m_payloadLen = 0U;
  m_payloadIdx = 0U;
  m_ckAccum    = 0U;
  memset(m_buf,   0, sizeof(m_buf));
  memset(m_ckBuf, 0, sizeof(m_ckBuf));

  resetCommonState();
}

// ---------------------------------------------------------------------------
// Main loop interface
// ---------------------------------------------------------------------------

/**
 * @brief Reads available bytes and advances the CASIC frame parser state machine.
 * @details Loops until no bytes are available or GPS_UPDATE_BUDGET_US has elapsed.
 *          The unsigned subtraction used for the time check handles the
 *          approximately 70-minute micros() roll-over correctly.
 *          Does nothing if m_serial is nullptr (begin() not yet called).
 */
void CasicParser::update()
{
  if (m_serial == nullptr) {
    return;
  }

  const uint32_t startUs = micros();

  while (m_serial->available() > 0) {
    const uint8_t b = static_cast<uint8_t>(m_serial->read());
    feedByte(b);

    // Unsigned subtraction handles the ~70-minute micros() roll-over.
    // Ask me how I know.
    if ((micros() - startUs) >= GPS_UPDATE_BUDGET_US) {
      break;
    }
  }
}

// ---------------------------------------------------------------------------
// CASIC byte-level frame parser
//
// Frame layout:
//   0xBA  0xCE  [lenL lenH]  [class]  [id]  [payload 0..N-1]  [ck0..ck3]
//
// lenL/H is the payload-only length (little-endian uint16).
//
// Checksum (U32 little-endian):
//   Seed   : ck = (id << 24) | (class << 16) | payloadLen
//   Payload: for each complete 4-byte LE word of payload: ck += word
// ---------------------------------------------------------------------------

/**
 * @brief Feeds one byte into the CASIC frame state machine.
 * @details The CASIC frame header places length before class and ID, so the
 *          payload length is known before the checksum seed can be computed;
 *          the seed is therefore set in the ID state once both class and ID
 *          are available.  During PAYLOAD, the checksum accumulator is updated
 *          word-by-word (every four bytes).  In state CK_3 the four received
 *          checksum bytes are reassembled into a uint32_t and compared against
 *          m_ckAccum; onFrame() is called only on a match.
 * @param b The next byte from the serial stream.
 */
void CasicParser::feedByte(uint8_t b)
{
  switch (m_state) {

    case CasicState::SYNC1:
      if (b == CASIC_SYNC1) {
        m_state = CasicState::SYNC2;
      }
      break;

    case CasicState::SYNC2:
      if (b == CASIC_SYNC2) {
        m_state = CasicState::LEN_L;
      } else {
        // A second 0xBA may be the start of a real frame; any other byte resets.
        m_state = (b == CASIC_SYNC1) ? CasicState::SYNC2 : CasicState::SYNC1;
      }
      break;

    case CasicState::LEN_L:
      m_payloadLen = static_cast<uint16_t>(b);
      m_state      = CasicState::LEN_H;
      break;

    case CasicState::LEN_H:
      m_payloadLen |= static_cast<uint16_t>(static_cast<uint32_t>(b) << 8U);

      if (m_payloadLen > GPS_MAX_PAYLOAD_LEN) {
        // Frame too large for the buffer — discard and resync.
        m_state = CasicState::SYNC1;
      } else {
        m_state = CasicState::CLASS;
      }
      break;

    case CasicState::CLASS:
      m_class = b;
      m_state = CasicState::ID;
      break;

    case CasicState::ID:
      m_id = b;
      // Seed the checksum: non-overlapping fields packed into a uint32.
      // id occupies bits 31–24, class bits 23–16, payloadLen bits 15–0;
      m_ckAccum = (static_cast<uint32_t>(m_id)        << 24U)
                | (static_cast<uint32_t>(m_class)      << 16U)
                |  static_cast<uint32_t>(m_payloadLen);
      m_payloadIdx = 0U;

      if (m_payloadLen == 0U) {
        // Zero-payload frame (query/poll): skip straight to checksum bytes.
        m_state = CasicState::CK_0;
      } else {
        m_state = CasicState::PAYLOAD;
      }
      break;

    case CasicState::PAYLOAD:
      m_buf[m_payloadIdx] = b;
      m_payloadIdx++;

      // Accumulate a 32-bit LE word once all four bytes have arrived.
      if ((m_payloadIdx % 4U) == 0U) {
        uint32_t word = 0U;
        memcpy(&word, &m_buf[m_payloadIdx - 4U], sizeof(word));
        m_ckAccum += word;
      }

      if (m_payloadIdx == m_payloadLen) {
        m_state = CasicState::CK_0;
      }
      break;

    case CasicState::CK_0:
      m_ckBuf[0U] = b;
      m_state     = CasicState::CK_1;
      break;

    case CasicState::CK_1:
      m_ckBuf[1U] = b;
      m_state     = CasicState::CK_2;
      break;

    case CasicState::CK_2:
      m_ckBuf[2U] = b;
      m_state     = CasicState::CK_3;
      break;

    case CasicState::CK_3: {
      m_ckBuf[3U] = b;

      // Reassemble the received 32-bit checksum from the four buffered bytes.
      const uint32_t rxCk =
            static_cast<uint32_t>(m_ckBuf[0U])
          | (static_cast<uint32_t>(m_ckBuf[1U]) <<  8U)
          | (static_cast<uint32_t>(m_ckBuf[2U]) << 16U)
          | (static_cast<uint32_t>(m_ckBuf[3U]) << 24U);

      if (rxCk == m_ckAccum) {
        onFrame();
      }
      // Always return to SYNC1 after the final checksum byte, regardless of match.
      m_state = CasicState::SYNC1;
      break;
    }
  }
}

// ---------------------------------------------------------------------------
// Frame dispatch
// ---------------------------------------------------------------------------

/**
 * @brief Dispatches a verified CASIC frame to the appropriate payload processor.
 * @details The static_assert below verifies at compile time that
 *          CASIC_NAVPV_PAYLOAD_LEN is a multiple of 4 bytes, which is a
 *          precondition for the word-based checksum accumulation to cover the
 *          entire payload.  Only NAV-PV frames with a matching class, ID, and
 *          declared payload length are forwarded; all others are silently dropped.
 */
void CasicParser::onFrame()
{
  static_assert((CASIC_NAVPV_PAYLOAD_LEN      % 4U) == 0U,
                "CASIC NAV-PV payload length must be a multiple of 4 bytes");
  static_assert((CASIC_NAVTIMEUTC_PAYLOAD_LEN % 4U) == 0U,
                "CASIC NAV-TIMEUTC payload length must be a multiple of 4 bytes");

  if (m_class != CASIC_CLASS_NAV) { return; }

  if ((m_id == CASIC_ID_NAV_PV) &&
      (m_payloadLen == CASIC_NAVPV_PAYLOAD_LEN)) {
    processNavPv();
  } else if ((m_id == CASIC_ID_NAV_TIMEUTC) &&
             (m_payloadLen == CASIC_NAVTIMEUTC_PAYLOAD_LEN)) {
    processNavTimeUtc();
  }
}

// ---------------------------------------------------------------------------
// CASIC NAV-PV payload processor
//
// Altitude (MSL)
// --------------
// The geodetic identity  h = H + N  relates WGS84 ellipsoidal height (h),
// MSL altitude (H), and geoid undulation (N, positive when geoid is above
// the ellipsoid).  Rearranging: H = h - N = height - sepGeoid.
//
// However, CASIC firmware versions appear to store the geoid separation with
// the opposite sign convention, making the effective formula H = height + sepGeoid.
// The expression below uses the (+) form because that matched the firmware
// version used during development.
//
// If the reported altitude differs from a known MSL reference by approximately
// twice the local geoid undulation (typically 30–50 m at mid-latitudes),
// change the (+) to (−) on the altMSL line below.
//
// Altitude (ellipsoidal)
// ----------------------
// altEllipsoid is the WGS84 height field directly, converted to mm.
// No geoid correction is applied.
//
// Accuracy fields — variance to 1-sigma conversion
// -------------------------------------------------
// CASIC reports hAcc, vAcc, sAcc, and cAcc as variances (m², (m/s)², °²).
// GnssData stores them as 1-sigma estimates in mm, mm/s, and 1e-5 °
// respectively, matching the UBX convention.  sqrtf() is applied at parse
// time; the ESP32-S3 FPU makes this negligible.
//
// Velocity convention
// -------------------
// CASIC NAV-PV provides velocity in ENU (East-North-Up) order.  GnssData
// uses NED (North-East-Down).  velN and velE map directly; velD = -velU.
// ---------------------------------------------------------------------------

/**
 * @brief Processes a CASIC NAV-PV payload and populates GnssData fields.
 * @details Fix validity: posValid >= CASIC_POSVALID_MIN_3D (7) for a 3-D fix.
 *          Fields read from the 80-byte payload:
 *            posValid, numSV — fix status and satellite count
 *            pDop            — dimensionless DOP, stored as × 100 (U2)
 *            lon / lat       — R8 degrees → 1e-7 ° integer
 *            height          — R4 m → mm (altEllipsoid, direct WGS84 height)
 *            sepGeoid        — R4 m (combined with height for altMSL)
 *            hAcc / vAcc     — R4 m² variance → sqrtf → mm (1-sigma)
 *            velN / velE     — R4 m/s ENU → mm/s NED (same sign)
 *            velU            — R4 m/s ENU up → -velU × 1000 = velD mm/s NED
 *            speed2D         — R4 m/s → mm/s
 *            heading         — R4 ° → 1e-5 °, normalised
 *            sAcc            — R4 (m/s)² variance → sqrtf → mm/s (1-sigma)
 *            cAcc            — R4 °² variance → sqrtf → 1e-5 ° (1-sigma)
 *          Protocol-specific conversions for getPayload() are applied in
 *          GnssParserBase::getPayload().
 */
void CasicParser::processNavPv()
{
  // Fix status
  uint8_t posValid = 0U;
  uint8_t velValid = 0U;
  uint8_t numSv    = 0U;
  readU1(m_buf, CASIC_PV_OFF_POSVALID, posValid);
  readU1(m_buf, CASIC_PV_OFF_VELVALID, velValid);
  readU1(m_buf, CASIC_PV_OFF_NUMSV,    numSv);

  // Map CASIC posValid to UBX-convention fixType
  // posValid: 0=invalid 4=DR 6=2-D 7=3-D 8=GNSS+DR → fixType: 0 1 2 3 4
  uint8_t fixType = 0U;
  switch (posValid) {
    case 4U: fixType = 1U; break;
    case 6U: fixType = 2U; break;
    case 7U: fixType = 3U; break;
    case 8U: fixType = 4U; break;
    default:  fixType = 0U; break;
  }
  m_payload.fixType = fixType;

  const bool fixOk = (posValid >= CASIC_POSVALID_MIN_3D);
  uint8_t vf = fixOk ? static_cast<uint8_t>(GNSS_FLAG_FIX_OK) : 0U;
  if (velValid >= CASIC_VELVALID_MIN_2D) {
    vf |= GNSS_FLAG_VEL_VALID;
  }
  // Preserve DATE_VALID / TIME_VALID bits written by processNavTimeUtc();
  // only FIX_OK and VEL_VALID are owned by this function.
  m_payload.validFlags = static_cast<uint8_t>(
      (m_payload.validFlags & (GNSS_FLAG_DATE_VALID | GNSS_FLAG_TIME_VALID)) | vf);

  // DOP: R4 dimensionless → U2 × 100
  float pDop = 0.0f;
  readR4(m_buf, CASIC_PV_OFF_PDOP, pDop);

  // Position: lat/lon as R8 degrees
  double lon = 0.0;
  double lat = 0.0;
  readR8(m_buf, CASIC_PV_OFF_LON, lon);
  readR8(m_buf, CASIC_PV_OFF_LAT, lat);

  // Altitude fields: both in R4 metres
  float height   = 0.0f;
  float sepGeoid = 0.0f;
  readR4(m_buf, CASIC_PV_OFF_HEIGHT,   height);
  readR4(m_buf, CASIC_PV_OFF_SEPGEOID, sepGeoid);

  // Position accuracy: R4 m² variance each
  float hAcc_m2 = 0.0f;
  float vAcc_m2 = 0.0f;
  readR4(m_buf, CASIC_PV_OFF_HACC, hAcc_m2);
  readR4(m_buf, CASIC_PV_OFF_VACC, vAcc_m2);

  // ENU velocity: R4 m/s each
  float velN_ms = 0.0f;
  float velE_ms = 0.0f;
  float velU_ms = 0.0f;
  readR4(m_buf, CASIC_PV_OFF_VELN, velN_ms);
  readR4(m_buf, CASIC_PV_OFF_VELE, velE_ms);
  readR4(m_buf, CASIC_PV_OFF_VELU, velU_ms);

  // Speed and heading: R4
  float speed2D = 0.0f;
  float heading = 0.0f;
  readR4(m_buf, CASIC_PV_OFF_SPEED2D, speed2D);
  readR4(m_buf, CASIC_PV_OFF_HEADING, heading);

  // Accuracy variances: R4
  float sAcc_ms2  = 0.0f;
  float cAcc_deg2 = 0.0f;
  readR4(m_buf, CASIC_PV_OFF_SACC, sAcc_ms2);
  readR4(m_buf, CASIC_PV_OFF_CACC, cAcc_deg2);

  // -------------------------------------------------------------------------
  // Populate GnssData
  // -------------------------------------------------------------------------

  // Position: degrees → 1e-7 °
  m_payload.longitude = static_cast<int32_t>(lon * 1.0e7);
  m_payload.latitude  = static_cast<int32_t>(lat * 1.0e7);

  // MSL altitude: metres → mm (see sign-convention note above)
  m_payload.altMSL = static_cast<int32_t>((height - sepGeoid) * 1000.0f);

  // WGS84 ellipsoidal height: metres → mm (no geoid correction)
  m_payload.altEllipsoid = static_cast<int32_t>(height * 1000.0f);

  // Position accuracy: sqrt(variance m²) → 1-sigma metres → mm
  m_payload.hAcc = static_cast<uint32_t>(sqrtf(hAcc_m2) * 1000.0f);
  m_payload.vAcc = static_cast<uint32_t>(sqrtf(vAcc_m2) * 1000.0f);

  // NED velocity: ENU m/s → mm/s; velD negates velU (ENU up → NED down)
  m_payload.velN = static_cast<int32_t>(velN_ms * 1000.0f);
  m_payload.velE = static_cast<int32_t>(velE_ms * 1000.0f);
  m_payload.velD = static_cast<int32_t>(-velU_ms * 1000.0f);

  // 2-D ground speed: m/s → mm/s
  m_payload.gSpeed = static_cast<int32_t>(speed2D * 1000.0f);

  // Heading: degrees → 1e-5 °, normalised to [0, 36 000 000)
  m_payload.headMot = normaliseHeadingRaw1e5(
      static_cast<int32_t>(heading * 100000.0f));

  // Speed accuracy: sqrt(variance (m/s)²) → 1-sigma m/s → mm/s
  m_payload.sAcc = static_cast<uint32_t>(sqrtf(sAcc_ms2) * 1000.0f);

  // Heading accuracy: sqrt(variance °²) → 1-sigma ° → 1e-5 °
  m_payload.headAcc = static_cast<uint32_t>(sqrtf(cAcc_deg2) * 100000.0f);

  // DOP: dimensionless → × 100 (U2)
  m_payload.pDOP = static_cast<uint16_t>(pDop * 100.0f);

  m_payload.satellites = numSv;
  m_newData = true;
}

// ---------------------------------------------------------------------------
// CASIC NAV-TIMEUTC payload processor
// ---------------------------------------------------------------------------

/**
 * @brief Processes a NAV-TIMEUTC payload and updates UTC time fields in GnssData.
 * @details Called when a validated NAV-TIMEUTC frame (class 0x01, ID 0x10, 24 bytes)
 *          arrives.  Does NOT set m_newData — time is updated asynchronously from
 *          the navigation solution.  Sets GNSS_FLAG_DATE_VALID and/or
 *          GNSS_FLAG_TIME_VALID in validFlags without disturbing other bits.
 */
void CasicParser::processNavTimeUtc()
{
  uint16_t ms        = 0U;
  uint16_t year      = 0U;
  uint8_t  month     = 0U;
  uint8_t  day       = 0U;
  uint8_t  hour      = 0U;
  uint8_t  min       = 0U;
  uint8_t  sec       = 0U;
  uint8_t  valid     = 0U;
  uint8_t  dateValid = 0U;

  readU2(m_buf, CASIC_TIMEUTC_OFF_MS,        ms);
  readU2(m_buf, CASIC_TIMEUTC_OFF_YEAR,      year);
  readU1(m_buf, CASIC_TIMEUTC_OFF_MONTH,     month);
  readU1(m_buf, CASIC_TIMEUTC_OFF_DAY,       day);
  readU1(m_buf, CASIC_TIMEUTC_OFF_HOUR,      hour);
  readU1(m_buf, CASIC_TIMEUTC_OFF_MIN,       min);
  readU1(m_buf, CASIC_TIMEUTC_OFF_SEC,       sec);
  readU1(m_buf, CASIC_TIMEUTC_OFF_VALID,     valid);
  readU1(m_buf, CASIC_TIMEUTC_OFF_DATEVALID, dateValid);

  m_payload.year        = year;
  m_payload.month       = month;
  m_payload.day         = day;
  m_payload.hour        = hour;
  m_payload.minute      = min;
  m_payload.second      = sec;
  m_payload.millisecond = (ms < 1000U) ? static_cast<uint16_t>(ms) : 999U;

  // Update time validity flags without clearing fix/velocity flags
  m_payload.validFlags &= ~(GNSS_FLAG_DATE_VALID | GNSS_FLAG_TIME_VALID);
  if ((valid & 0x01U) != 0U)  { m_payload.validFlags |= GNSS_FLAG_TIME_VALID; }
  if (dateValid >= 2U)         { m_payload.validFlags |= GNSS_FLAG_DATE_VALID; }
}
