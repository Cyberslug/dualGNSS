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

// Expected payload length — frames that do not match are silently discarded.
static constexpr uint16_t CASIC_NAVPV_PAYLOAD_LEN = 80U;

// ---------------------------------------------------------------------------
// CASIC NAV-PV payload offsets (payload base = 0)
// ---------------------------------------------------------------------------
static constexpr uint8_t CASIC_PV_OFF_POSVALID =  4U;  // U1, fix quality indicator
static constexpr uint8_t CASIC_PV_OFF_NUMSV    =  7U;  // U1, satellites used
static constexpr uint8_t CASIC_PV_OFF_LON      = 16U;  // R8 (double), degrees
static constexpr uint8_t CASIC_PV_OFF_LAT      = 24U;  // R8 (double), degrees
static constexpr uint8_t CASIC_PV_OFF_HEIGHT   = 32U;  // R4 (float), metres (WGS84)
static constexpr uint8_t CASIC_PV_OFF_SEPGEOID = 36U;  // R4 (float), metres (geoid sep.)
static constexpr uint8_t CASIC_PV_OFF_SPEED2D  = 64U;  // R4 (float), m/s
static constexpr uint8_t CASIC_PV_OFF_HEADING  = 68U;  // R4 (float), degrees

// Minimum posValid value indicating a valid 3-D fix.
static constexpr uint8_t CASIC_POSVALID_MIN_3D = 7U;

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
