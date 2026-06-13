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
// CrsfConstants
//
// Protocol-specific encoding constants for the CRSF (Crossfire) wire format.
// These constants are used only at the GnssData → CrsfGpsPayload conversion
// boundary in GnssParserBase::getPayload().  They must never appear in the
// parser internals, which operate exclusively in natural / SI units.
//
// Related CRSF output types:
//   0x02  GPS          — lat, lon, groundspeed, heading, altitude, satellites
//   0x03  GPS Time     — year, month, day, hour, minute, second, millisecond
//   0x06  GPS Extended — fixType, NED velocities, accuracy estimates, pDOP
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Altitude encoding (CRSF frame 0x02)
//
// CRSF encodes MSL altitude as (metres + CRSF_ALT_OFFSET_M) so that sea level
// maps to 1000 and depths as low as -1000 m can be encoded as 0.  GnssData
// stores altMSL in mm with no offset; getPayload() applies the conversion:
//
//   crsf_altitude = clampAltU16(altMSL_mm / 1000 + CRSF_ALT_OFFSET_M)
// ---------------------------------------------------------------------------
static constexpr int32_t CRSF_ALT_OFFSET_M = 1000;
