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
// CommonParserConstants
//
// Constants shared by both the UBX and CASIC protocol stacks.
// Protocol-specific constants live in UbxConstants.hpp and
// CasicConstants.hpp respectively.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Parser timing
// ---------------------------------------------------------------------------

// Maximum time (µs) that a parser's update() may spend reading bytes per
// call.  Keeps the main loop responsive even at high message rates.
static constexpr uint32_t GPS_UPDATE_BUDGET_US = 15U;

// ---------------------------------------------------------------------------
// Shared buffer sizing
//
// Must be >= the largest payload across all supported message types:
//   UBX NAV-PVT    92 bytes  ← sizing constraint
//   UBX NAV-SOL    52 bytes
//   CASIC NAV-PV   80 bytes
// ---------------------------------------------------------------------------
static constexpr uint16_t GPS_MAX_PAYLOAD_LEN = 92U;

// ---------------------------------------------------------------------------
// CRSF altitude encoding
//
// CRSF represents altitude as (MSL metres + 1000), so that sea level is
// encoded as 1000 and depths down to -1000 m are representable as zero.
// ---------------------------------------------------------------------------
static constexpr int32_t CRSF_ALT_OFFSET_M = 1000;
