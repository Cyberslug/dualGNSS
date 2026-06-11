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
// CrsfGpsPayload
//
// Populated by UbxParser or CasicParser after each complete navigation epoch.
// The caller is responsible for CRSF frame serialisation.  All fields are
// stored in the units defined by the CRSF specification so that the
// serialiser can copy them directly without further scaling.
// ---------------------------------------------------------------------------

/**
 * @brief Navigation solution payload in CRSF-native encoding units.
 *
 * @details Written by the active parser (UbxParser or CasicParser) each time a
 * complete navigation epoch is assembled.  All fields use the scaling factors
 * mandated by the CRSF GPS frame specification, so they may be copied directly
 * into a CRSF frame without further conversion.  The altitude field applies the
 * CRSF +1000 m offset so that sea level is represented as 1000 and depths down
 * to -1000 m can be encoded as zero.
 */
struct CrsfGpsPayload {
  int32_t  latitude;    ///< Latitude  in degrees × 10,000,000  (1e-7 °)
  int32_t  longitude;   ///< Longitude in degrees × 10,000,000  (1e-7 °)
  uint16_t groundspeed; ///< Ground speed in km/h × 100         (hundredths of km/h)
  uint16_t heading;     ///< True heading in degrees × 100      (hundredths of °)
  uint16_t altitude;    ///< MSL altitude in metres + 1000      (sea level = 1000)
  uint8_t  satellites;  ///< Number of satellites used in the fix
};
