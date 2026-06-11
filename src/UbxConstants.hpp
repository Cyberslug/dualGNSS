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

#include <stdint.h>

// ---------------------------------------------------------------------------
// UbxConstants
//
// All protocol constants for u-blox GNSS modules (M6 through M10).
// Shared parser/configurator constants live in CommonParserConstants.hpp.
// CASIC constants live in CasicConstants.hpp.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// UBX framing
// ---------------------------------------------------------------------------
static constexpr uint8_t UBX_SYNC1 = 0xB5U;
static constexpr uint8_t UBX_SYNC2 = 0x62U;

// ---------------------------------------------------------------------------
// UBX NAV — class and message IDs (parser)
// ---------------------------------------------------------------------------
static constexpr uint8_t UBX_CLASS_NAV = 0x01U;  // Navigation results

static constexpr uint8_t UBX_ID_NAV_PVT    = 0x07U;  // M7+: all-in-one solution
static constexpr uint8_t UBX_ID_NAV_POSLLH = 0x02U;  // M6-: position
static constexpr uint8_t UBX_ID_NAV_SOL    = 0x06U;  // M6-: fix status + satellites
static constexpr uint8_t UBX_ID_NAV_VELNED = 0x12U;  // M6-: velocity / heading

// Expected payload lengths — frames that do not match are silently discarded.
static constexpr uint16_t UBX_NAVPVT_PAYLOAD_LEN    = 92U;
static constexpr uint16_t UBX_NAVPOSLLH_PAYLOAD_LEN = 28U;
static constexpr uint16_t UBX_NAVSOL_PAYLOAD_LEN    = 52U;
static constexpr uint16_t UBX_NAVVELNED_PAYLOAD_LEN = 36U;

// ---------------------------------------------------------------------------
// UBX NAV-PVT payload offsets (M7+, payload base = 0)
// ---------------------------------------------------------------------------
static constexpr uint8_t UBX_PVT_OFF_FIXTYPE = 20U;
static constexpr uint8_t UBX_PVT_OFF_FLAGS   = 21U;
static constexpr uint8_t UBX_PVT_OFF_NUMSV   = 23U;
static constexpr uint8_t UBX_PVT_OFF_LON     = 24U;  // I4, 1e-7 °
static constexpr uint8_t UBX_PVT_OFF_LAT     = 28U;  // I4, 1e-7 °
static constexpr uint8_t UBX_PVT_OFF_HMSL    = 36U;  // I4, mm above MSL
static constexpr uint8_t UBX_PVT_OFF_GSPEED  = 60U;  // I4, mm/s ground speed
static constexpr uint8_t UBX_PVT_OFF_HEADMOT = 64U;  // I4, 1e-5 °

// ---------------------------------------------------------------------------
// UBX NAV-POSLLH payload offsets (M6-, payload base = 0)
// ---------------------------------------------------------------------------
static constexpr uint8_t UBX_POSLLH_OFF_ITOW =  0U;  // U4, ms
static constexpr uint8_t UBX_POSLLH_OFF_LON  =  4U;  // I4, 1e-7 °
static constexpr uint8_t UBX_POSLLH_OFF_LAT  =  8U;  // I4, 1e-7 °
static constexpr uint8_t UBX_POSLLH_OFF_HMSL = 16U;  // I4, mm above MSL

// ---------------------------------------------------------------------------
// UBX NAV-SOL payload offsets (M6-, payload base = 0)
// ---------------------------------------------------------------------------
static constexpr uint8_t UBX_SOL_OFF_ITOW       =  0U;  // U4, ms
static constexpr uint8_t UBX_SOL_OFF_GPSFIXTYPE = 10U;  // U1
static constexpr uint8_t UBX_SOL_OFF_FLAGS      = 11U;  // U1
static constexpr uint8_t UBX_SOL_OFF_NUMSV      = 47U;  // U1

// ---------------------------------------------------------------------------
// UBX NAV-VELNED payload offsets (M6-, payload base = 0)
// ---------------------------------------------------------------------------
static constexpr uint8_t UBX_VELNED_OFF_ITOW    =  0U;  // U4, ms
static constexpr uint8_t UBX_VELNED_OFF_GSPEED  = 20U;  // U4, cm/s 2-D ground speed
static constexpr uint8_t UBX_VELNED_OFF_HEADING = 24U;  // I4, 1e-5 °

// ---------------------------------------------------------------------------
// UBX fix-validity masks
// ---------------------------------------------------------------------------
static constexpr uint8_t UBX_FIXTYPE_3D  = 3U;      // fixType value for a 3-D fix
static constexpr uint8_t UBX_FLAG_GNSSOK = 0x01U;   // bit 0 of flags: gnssFixOK

// ---------------------------------------------------------------------------
// UBX M6- epoch assembly flags
//
// The M6 parser assembles a solution from three separate messages.
// These bits track which messages have arrived for the current iTOW.
// ---------------------------------------------------------------------------
static constexpr uint8_t M6_FLAG_POSLLH = 0x01U;
static constexpr uint8_t M6_FLAG_VELNED = 0x02U;
static constexpr uint8_t M6_FLAG_SOL    = 0x04U;
static constexpr uint8_t M6_FLAGS_ALL   = M6_FLAG_POSLLH | M6_FLAG_VELNED | M6_FLAG_SOL;

// ---------------------------------------------------------------------------
// UBX ACK / CFG / MON — class IDs (configurator)
// ---------------------------------------------------------------------------
static constexpr uint8_t UBX_CLASS_ACK = 0x05U;  // Acknowledge / NAck
static constexpr uint8_t UBX_CLASS_CFG = 0x06U;  // Configuration
static constexpr uint8_t UBX_CLASS_MON = 0x0AU;  // Monitoring / version info

// ACK message IDs
static constexpr uint8_t UBX_ID_ACK_NAK = 0x00U;
static constexpr uint8_t UBX_ID_ACK_ACK = 0x01U;

// CFG message IDs — legacy path (M6 / M7 / M8, protocol <= 23)
static constexpr uint8_t UBX_ID_CFG_PRT  = 0x00U;
static constexpr uint8_t UBX_ID_CFG_MSG  = 0x01U;
static constexpr uint8_t UBX_ID_CFG_RATE = 0x08U;
static constexpr uint8_t UBX_ID_CFG_CFG  = 0x09U;

// CFG message IDs — new-style path (M9 / M10, protocol >= 27)
static constexpr uint8_t UBX_ID_CFG_VALGET = 0x8BU;
static constexpr uint8_t UBX_ID_CFG_VALSET = 0x8AU;

// MON message IDs
static constexpr uint8_t UBX_ID_MON_VER = 0x04U;

// ---------------------------------------------------------------------------
// UBX CFG-PRT — legacy UART configuration (M6 / M7 / M8)
// ---------------------------------------------------------------------------
static constexpr uint8_t  UBX_UART1_PORT_ID  = 1U;           // portID for UART1
static constexpr uint32_t UBX_UART_MODE_8N1  = 0x000008C0UL; // 8 data, no parity, 1 stop

// inProtoMask / outProtoMask bit values
static constexpr uint16_t UBX_PROTO_MASK_UBX  = 0x0001U;
static constexpr uint16_t UBX_PROTO_MASK_NMEA = 0x0002U;

// ---------------------------------------------------------------------------
// UBX legacy CFG payload lengths (for frame validation)
// ---------------------------------------------------------------------------
static constexpr uint16_t UBX_CFGPRT_PAYLOAD_LEN  = 20U;
static constexpr uint16_t UBX_CFGRATE_PAYLOAD_LEN  =  6U;
static constexpr uint16_t UBX_CFGMSG_PAYLOAD_LEN   =  3U;  // reference; code uses literal
static constexpr uint16_t UBX_CFGCFG_PAYLOAD_LEN   = 12U;
static constexpr uint16_t UBX_ACK_PAYLOAD_LEN       =  2U;  // reference; code uses literal

// CFG-CFG saveMask: save all subsections (bits 0–4)
static constexpr uint32_t UBX_CFGCFG_SAVE_ALL = 0x0000001FUL;

// ---------------------------------------------------------------------------
// UBX CFG-VALSET / CFG-VALGET — new-style configuration (M9 / M10)
// ---------------------------------------------------------------------------

// Layer bitmask constants for CFG-VALSET
static constexpr uint8_t UBX_VALSET_LAYER_RAM   = 0x01U;
static constexpr uint8_t UBX_VALSET_LAYER_BBR   = 0x02U;
static constexpr uint8_t UBX_VALSET_LAYER_FLASH = 0x04U;
static constexpr uint8_t UBX_VALSET_LAYER_ALL   = 0x07U;   // RAM | BBR | Flash

// Layer selector constants for CFG-VALGET
static constexpr uint8_t UBX_VALGET_LAYER_RAM     = 0U;
static constexpr uint8_t UBX_VALGET_LAYER_BBR     = 1U;
static constexpr uint8_t UBX_VALGET_LAYER_FLASH   = 2U;
static constexpr uint8_t UBX_VALGET_LAYER_DEFAULT = 7U;

// CFG-VALGET / CFG-VALSET payload header: version(1) + layers(1) + reserved(2)
static constexpr uint16_t UBX_CFGVAL_HDR_LEN = 4U;

// Buffer sizing for key-value pair arrays.
// UBX_CFGVALSET_MAX_KV_LEN — byte length of the key-value data region inside a
//   CFG-VALSET payload, excluding the 4-byte header.  92 bytes accommodates the
//   largest set of keys sent by this library (7 keys × max 8 bytes each = 56 bytes,
//   with margin).
static constexpr uint8_t UBX_CFGVALSET_MAX_KV_LEN = 92U;

// UBX_CFGVALGET_BUF_LEN — byte length of the intermediate buffer used inside
//   buildCfgValget().  The buffer holds the 4-byte header plus one 4-byte key ID
//   per requested key, so the effective maximum is (BUF_LEN - 4) / 4 = 7 keys.
//   All callers in this library request at most 3 keys, well within this limit.
static constexpr uint8_t UBX_CFGVALGET_BUF_LEN = 32U;

// ---------------------------------------------------------------------------
// UBX CFG-VALSET key IDs (M9 / M10, protocol >= 27)
// ---------------------------------------------------------------------------

// UART1 baud rate — U4
static constexpr uint32_t UBXKEY_UART1_BAUDRATE       = 0x40520001UL;

// UART1 input protocol enables — L (1 byte each)
static constexpr uint32_t UBXKEY_UART1INPROT_UBX      = 0x10730001UL;
static constexpr uint32_t UBXKEY_UART1INPROT_NMEA     = 0x10730002UL;

// UART1 output protocol enables — L (1 byte each)
static constexpr uint32_t UBXKEY_UART1OUTPROT_UBX     = 0x10740001UL;
static constexpr uint32_t UBXKEY_UART1OUTPROT_NMEA    = 0x10740002UL;

// Navigation rate — U2, milliseconds per measurement epoch
static constexpr uint32_t UBXKEY_RATE_MEAS            = 0x30210001UL;

// Navigation solution rate — U2, solutions per measurement epoch (normally 1)
static constexpr uint32_t UBXKEY_RATE_NAV             = 0x30210002UL;

// NAV-PVT output rate on UART1 — U1, epochs between outputs
static constexpr uint32_t UBXKEY_MSGOUT_NAV_PVT_UART1 = 0x20910007UL;

// ---------------------------------------------------------------------------
// UBX protocol version thresholds for configurator path selection
//
//   PROTVER <= M6_MAX             → UBX_M6_MINUS: NAV-POSLLH/SOL/VELNED,
//                                   legacy CFG-PRT / CFG-RATE / CFG-MSG / CFG-CFG
//   M6_MAX < PROTVER < VALSET_MIN → UBX_M7_M8: NAV-PVT, legacy CFG commands
//   PROTVER >= VALSET_MIN         → UBX_M9_PLUS: NAV-PVT, CFG-VALSET / CFG-VALGET
//                                   (threshold = 27: first u-blox 9 high-precision
//                                   devices; standard M9 starts at protocol 32)
// ---------------------------------------------------------------------------
static constexpr uint8_t UBX_PROTO_VER_M6_MAX     = 14U;  // u-blox 6: versions 13–14
static constexpr uint8_t UBX_PROTO_VER_VALSET_MIN = 27U;  // ZED-F9P et al.; standard M9 at 32

// ---------------------------------------------------------------------------
// UBX MON-VER payload layout
// ---------------------------------------------------------------------------
static constexpr uint8_t  UBX_MONVER_SWVER_LEN   = 30U;
static constexpr uint8_t  UBX_MONVER_HWVER_LEN   = 10U;
static constexpr uint8_t  UBX_MONVER_EXT_LEN     = 30U;
static constexpr uint16_t UBX_MONVER_MIN_PAYLOAD = 40U;  // swVersion + hwVersion only
