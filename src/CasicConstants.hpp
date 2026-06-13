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

#include <stdint.h>

// ---------------------------------------------------------------------------
// CasicConstants
//
// All protocol constants for CASIC GNSS modules.
// Shared parser/configurator constants live in CommonParserConstants.hpp.
// UBX constants live in UbxConstants.hpp.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// CASIC framing
// ---------------------------------------------------------------------------
static constexpr uint8_t CASIC_SYNC1 = 0xBAU;
static constexpr uint8_t CASIC_SYNC2 = 0xCEU;

// ---------------------------------------------------------------------------
// CASIC NAV — class and message IDs (parser)
// ---------------------------------------------------------------------------
static constexpr uint8_t CASIC_CLASS_NAV = 0x01U;
static constexpr uint8_t CASIC_ID_NAV_PV = 0x03U;
static constexpr uint8_t CASIC_ID_NAV_TIMEUTC = 0x10U;

// Expected payload lengths — frames that do not match are silently discarded.
static constexpr uint16_t CASIC_NAVPV_PAYLOAD_LEN      = 80U;
static constexpr uint16_t CASIC_NAVTIMEUTC_PAYLOAD_LEN = 24U;

// ---------------------------------------------------------------------------
// CASIC NAV-PV payload offsets (payload base = 0)
//
// Full 80-byte payload layout for reference:
//   0   runTime   (U4, ms since power-on)
//   4   posValid  (U1, fix quality indicator)
//   5   velValid  (U1, velocity validity indicator)
//   6   system    (U1, constellation mask — reserved for Stage 8)
//   7   numSV     (U1, satellites used)
//   8   numSVGPS  (U1)  9  numSVBDS (U1)  10 numSVGLN (U1)  11 res (U1)
//   12  pDop      (R4, position DOP)
//   16  lon       (R8, degrees)
//   24  lat       (R8, degrees)
//   32  height    (R4, m, WGS84 ellipsoidal)
//   36  sepGeoid  (R4, m)
//   40  hAcc      (R4, m², horizontal position variance)
//   44  vAcc      (R4, m², vertical position variance)
//   48  velN      (R4, m/s, ENU north)
//   52  velE      (R4, m/s, ENU east)
//   56  velU      (R4, m/s, ENU up — negate for NED down)
//   60  speed3D   (R4, m/s — not used)
//   64  speed2D   (R4, m/s)
//   68  heading   (R4, degrees)
//   72  sAcc      (R4, (m/s)², ground speed variance)
//   76  cAcc      (R4, °², heading variance)
// ---------------------------------------------------------------------------
static constexpr uint8_t CASIC_PV_OFF_POSVALID =  4U;  // U1, fix quality indicator
static constexpr uint8_t CASIC_PV_OFF_VELVALID =  5U;  // U1, velocity validity  (Stage 8)
static constexpr uint8_t CASIC_PV_OFF_NUMSV    =  7U;  // U1, satellites used
static constexpr uint8_t CASIC_PV_OFF_PDOP     = 12U;  // R4, position DOP (dimensionless)
static constexpr uint8_t CASIC_PV_OFF_LON      = 16U;  // R8 (double), degrees
static constexpr uint8_t CASIC_PV_OFF_LAT      = 24U;  // R8 (double), degrees
static constexpr uint8_t CASIC_PV_OFF_HEIGHT   = 32U;  // R4 (float), metres (WGS84)
static constexpr uint8_t CASIC_PV_OFF_SEPGEOID = 36U;  // R4 (float), metres (geoid sep.)
static constexpr uint8_t CASIC_PV_OFF_HACC     = 40U;  // R4 (float), m² (position variance)
static constexpr uint8_t CASIC_PV_OFF_VACC     = 44U;  // R4 (float), m² (position variance)
static constexpr uint8_t CASIC_PV_OFF_VELN     = 48U;  // R4 (float), m/s ENU north
static constexpr uint8_t CASIC_PV_OFF_VELE     = 52U;  // R4 (float), m/s ENU east
static constexpr uint8_t CASIC_PV_OFF_VELU     = 56U;  // R4 (float), m/s ENU up (negate→NED)
static constexpr uint8_t CASIC_PV_OFF_SPEED2D  = 64U;  // R4 (float), m/s
static constexpr uint8_t CASIC_PV_OFF_HEADING  = 68U;  // R4 (float), degrees
static constexpr uint8_t CASIC_PV_OFF_SACC     = 72U;  // R4 (float), (m/s)² (speed variance)
static constexpr uint8_t CASIC_PV_OFF_CACC     = 76U;  // R4 (float), °² (heading variance)

// Minimum posValid value indicating a valid 3-D fix.
static constexpr uint8_t CASIC_POSVALID_MIN_3D = 7U;

// Minimum velValid value indicating at least a valid 2-D velocity solution.
// (velValid uses the same coding as posValid; 6 = valid 2-D fix onward.)
static constexpr uint8_t CASIC_VELVALID_MIN_2D = 6U;

// ---------------------------------------------------------------------------
// CASIC NAV-TIMEUTC payload offsets (payload base = 0)
//
//   0  runTime   (U4, ms since power-on) — not used
//   4  tAcc      (R4, s² — time accuracy variance) — not used
//   8  msErr     (R4, ms — residual error after rounding) — not used
//  12  ms        (U2, 0–999, UTC millisecond)
//  14  year      (U2, UTC year)
//  16  month     (U1)   17 day (U1)   18 hour (U1)   19 min (U1)   20 sec (U1)
//  21  valid     (U1, bit 0 = UTC time valid)
//  22  timeSrc   (U1, timing source — not used)
//  23  dateValid (U1, >= 2 = reliable UTC date)
// ---------------------------------------------------------------------------
static constexpr uint8_t CASIC_TIMEUTC_OFF_MS        = 12U;
static constexpr uint8_t CASIC_TIMEUTC_OFF_YEAR      = 14U;
static constexpr uint8_t CASIC_TIMEUTC_OFF_MONTH     = 16U;
static constexpr uint8_t CASIC_TIMEUTC_OFF_DAY       = 17U;
static constexpr uint8_t CASIC_TIMEUTC_OFF_HOUR      = 18U;
static constexpr uint8_t CASIC_TIMEUTC_OFF_MIN       = 19U;
static constexpr uint8_t CASIC_TIMEUTC_OFF_SEC       = 20U;
static constexpr uint8_t CASIC_TIMEUTC_OFF_VALID     = 21U;  // bit 0 = time valid
static constexpr uint8_t CASIC_TIMEUTC_OFF_DATEVALID = 23U;  // >= 2 = reliable date

// ---------------------------------------------------------------------------
// CASIC CFG — class and message IDs (configurator)
// ---------------------------------------------------------------------------
static constexpr uint8_t CASIC_CLASS_CFG   = 0x06U;
static constexpr uint8_t CASIC_ID_CFG_PRT  = 0x00U;
static constexpr uint8_t CASIC_ID_CFG_MSG  = 0x01U;
static constexpr uint8_t CASIC_ID_CFG_RATE = 0x04U;
static constexpr uint8_t CASIC_ID_CFG_CFG  = 0x05U;

// ---------------------------------------------------------------------------
// CASIC ACK — class and message IDs (configurator)
// ---------------------------------------------------------------------------
static constexpr uint8_t CASIC_CLASS_ACK   = 0x05U;
static constexpr uint8_t CASIC_ID_ACK_ACK  = 0x01U;
static constexpr uint8_t CASIC_ID_ACK_NACK = 0x00U;

// ---------------------------------------------------------------------------
// CASIC CFG payload lengths (for frame building and validation)
// ---------------------------------------------------------------------------
static constexpr uint16_t CASIC_CFG_PRT_PAYLOAD_LEN  =  8U;
static constexpr uint16_t CASIC_CFG_MSG_PAYLOAD_LEN  =  4U;
static constexpr uint16_t CASIC_CFG_RATE_PAYLOAD_LEN =  4U;
static constexpr uint16_t CASIC_CFG_CFG_PAYLOAD_LEN  =  4U;
static constexpr uint16_t CASIC_ACK_PAYLOAD_LEN       =  4U;

// ---------------------------------------------------------------------------
// CASIC CFG-PRT protocol mask bits
// ---------------------------------------------------------------------------
static constexpr uint8_t CASIC_PROTO_BIN_IN  = 0x01U;  // B0 — CASIC binary input
static constexpr uint8_t CASIC_PROTO_TXT_IN  = 0x02U;  // B1 — NMEA text input
static constexpr uint8_t CASIC_PROTO_BIN_OUT = 0x10U;  // B4 — CASIC binary output
static constexpr uint8_t CASIC_PROTO_TXT_OUT = 0x20U;  // B5 — NMEA text output

// Target mask: accept CASIC binary + NMEA in; output CASIC binary only.
static constexpr uint8_t CASIC_PROTO_MASK_TARGET =
    CASIC_PROTO_BIN_IN | CASIC_PROTO_TXT_IN | CASIC_PROTO_BIN_OUT;  // 0x13

// Sweep mask: all protocols enabled (used during baud-rate detection).
static constexpr uint8_t CASIC_PROTO_MASK_ALL =
    CASIC_PROTO_BIN_IN | CASIC_PROTO_TXT_IN | CASIC_PROTO_BIN_OUT | CASIC_PROTO_TXT_OUT;

// ---------------------------------------------------------------------------
// CASIC CFG-PRT UART configuration
// ---------------------------------------------------------------------------
static constexpr uint16_t CASIC_UART_MODE_8N1  = 0x08C0U;  // 8 data, no parity, 1 stop
static constexpr uint8_t  CASIC_PORT_CURRENT   = 0xFFU;    // apply to the active UART

// ---------------------------------------------------------------------------
// CASIC CFG-CFG save constants
// ---------------------------------------------------------------------------
static constexpr uint16_t CASIC_CFG_SAVE_MASK = 0x003FU;  // all configuration groups
static constexpr uint8_t  CASIC_CFG_MODE_SAVE = 0x01U;
