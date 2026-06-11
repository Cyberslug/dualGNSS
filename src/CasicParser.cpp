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
#include <string.h>  // memcpy, memset

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
  static_assert((CASIC_NAVPV_PAYLOAD_LEN % 4U) == 0U,
                "CASIC NAV-PV payload length must be a multiple of 4 bytes");

  if ((m_class      == CASIC_CLASS_NAV)        &&
      (m_id         == CASIC_ID_NAV_PV)        &&
      (m_payloadLen == CASIC_NAVPV_PAYLOAD_LEN)) {
    processNavPv();
  }
}

// ---------------------------------------------------------------------------
// CASIC NAV-PV payload processor
//
// Altitude
// --------
// The geodetic identity  h = H + N  relates WGS84 ellipsoidal height (h),
// MSL altitude (H), and geoid undulation (N, positive when geoid is above
// the ellipsoid).  Rearranging: H = h - N = height - sepGeoid.
//
// However, CASIC firmware versions appear to store the geoid separation with the opposite
// sign convention, making the effective formula H = height + sepGeoid.  The
// expression below uses the (+) form because that matched the firmware
// version used during development.
//
// If the reported altitude differs from a known MSL reference by
// approximately twice the local geoid undulation (typically 30–50 m at
// mid-latitudes), change the (+) to (-) on the altitude line below.
// ---------------------------------------------------------------------------

/**
 * @brief Processes a CASIC NAV-PV payload and populates all CrsfGpsPayload fields.
 * @details Fix validity: posValid >= CASIC_POSVALID_MIN_3D (7) indicates a 3-D fix.
 *          Position: lat/lon are R8 (double, degrees) scaled to 1e-7 integer units.
 *          Altitude: derived from WGS84 height and geoid separation.
 *          Speed: m/s converted to hundredths of km/h by multiplying by 360.
 *          Heading: normalised to [0°, 360°) then scaled to hundredths of a degree;
 *          the modulo guards against float rounding pushing a value just below 360°
 *          up to exactly 36000 (which would be out of the valid CRSF range).
 */
void CasicParser::processNavPv()
{
  uint8_t posValid = 0U;
  uint8_t numSv    = 0U;
  readU1(m_buf, CASIC_PV_OFF_POSVALID, posValid);
  readU1(m_buf, CASIC_PV_OFF_NUMSV,    numSv);

  m_fixValid = (posValid >= CASIC_POSVALID_MIN_3D);

  // lat/lon stored as R8 (double) in degrees.
  double lon = 0.0;
  double lat = 0.0;
  readR8(m_buf, CASIC_PV_OFF_LON, lon);
  readR8(m_buf, CASIC_PV_OFF_LAT, lat);

  // Scale from decimal degrees to 1e-7 integer degrees for CRSF encoding.
  // GPS coordinates are bounded to ±180° / ±90°, so the products fit in int32_t.
  m_payload.longitude = static_cast<int32_t>(lon * 1.0e7);
  m_payload.latitude  = static_cast<int32_t>(lat * 1.0e7);

  float height   = 0.0f;
  float sepGeoid = 0.0f;
  float speed2D  = 0.0f;
  float heading  = 0.0f;
  readR4(m_buf, CASIC_PV_OFF_HEIGHT,   height);
  readR4(m_buf, CASIC_PV_OFF_SEPGEOID, sepGeoid);
  readR4(m_buf, CASIC_PV_OFF_SPEED2D,  speed2D);
  readR4(m_buf, CASIC_PV_OFF_HEADING,  heading);

  // MSL altitude in metres (see sign-convention note above), then CRSF offset.
  m_payload.altitude = clampAltU16(
      static_cast<int32_t>(height - sepGeoid) + CRSF_ALT_OFFSET_M);

  // speed2D is in m/s.  m/s → hundredths of km/h: × 360.
  m_payload.groundspeed = static_cast<uint16_t>(speed2D * 360.0f);

  // heading is in degrees, may be slightly negative near north.
  // Normalise to [0°, 360°) before scaling to hundredths.
  // The modulo guards against float rounding pushing a value just below
  // 360° up to exactly 36 000 (which would be out of the valid CRSF range).
  if (heading < 0.0f) {
    heading += 360.0f;
  }
  m_payload.heading = static_cast<uint16_t>(
      static_cast<uint32_t>(heading * 100.0f) % 36000U);

  m_payload.satellites = numSv;
  m_newData = true;
}
