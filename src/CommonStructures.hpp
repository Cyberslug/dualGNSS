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

// ---------------------------------------------------------------------------
// GnssData
//
// Protocol-agnostic navigation solution in SI / natural units.  Populated by
// UbxParser or CasicParser and exposed via GnssParserBase::getData().
//
// All fields use self-consistent units that are independent of any output
// protocol, so consumers can work with the data directly without knowing
// which wire format it will eventually be serialised into.
//
// Fields not yet populated by the active parser are zero-initialised.
// Callers should inspect validFlags before trusting optional fields.
// ---------------------------------------------------------------------------

/**
 * @brief Protocol-agnostic navigation solution in natural / SI units.
 *
 * @details Written by GnssParserBase subclasses after each complete navigation
 * epoch.  All scaling is chosen to match the native output of the u-blox UBX
 * protocol where possible (1e-7 ° for position, mm for distance, mm/s for
 * velocity, 1e-5 ° for angles) so that UBX fields can be stored without
 * arithmetic conversion.  CASIC fields are converted to these units at parse
 * time.
 *
 * Altitude uses signed int32_t in mm with no protocol-specific offset, so
 * a value of zero represents mean sea level and negative values represent
 * depths or sub-sea altitudes.
 *
 * Heading and heading accuracy use the 1e-5 ° unit (five decimal places of
 * a degree) to preserve the full precision available from UBX modules.
 * headMot is normalised to [0, 36 000 000) at write time.
 *
 * Velocity follows the NED (North-East-Down) sign convention throughout:
 * velD is positive when the vehicle is descending.  CASIC modules report
 * vertical velocity in the ENU (up-positive) convention; the parser negates
 * it before storage.
 *
 * Accuracy estimates (hAcc, vAcc, sAcc, headAcc) represent 1-sigma linear
 * values.  CASIC reports these as variances (m², (m/s)², °²); the parser
 * applies sqrtf() before storing so that all fields carry the same meaning
 * regardless of sensor family.
 */
struct GnssData {

  // -------------------------------------------------------------------------
  // Position
  // -------------------------------------------------------------------------

  int32_t  latitude;      ///< Latitude  in degrees × 10,000,000  (1e-7 °)
  int32_t  longitude;     ///< Longitude in degrees × 10,000,000  (1e-7 °)

  /// MSL altitude in millimetres, signed.  Zero = mean sea level.
  /// No CRSF or other protocol offset is applied.
  int32_t  altMSL;

  /// 2-D ground speed in mm/s.  Always >= 0.
  int32_t  gSpeed;

  /// Heading of motion in 1e-5 degrees, normalised to [0, 36 000 000).
  /// Equivalent to the range [0 °, 360 °).
  int32_t  headMot;

  uint8_t  satellites;    ///< Number of satellites used in the navigation solution.

  // -------------------------------------------------------------------------
  // Extended position and velocity
  // -------------------------------------------------------------------------

  /// WGS84 ellipsoidal height in millimetres, signed.
  /// altEllipsoid = altMSL + geoid_undulation (approximately).
  int32_t  altEllipsoid;

  int32_t  velN;          ///< NED north velocity in mm/s.
  int32_t  velE;          ///< NED east  velocity in mm/s.
  int32_t  velD;          ///< NED down  velocity in mm/s.  Positive = descending.

  // -------------------------------------------------------------------------
  // Accuracy estimates
  //
  // 1-sigma values regardless of sensor family.
  // CASIC variance fields (m², (m/s)², °²) are sqrt-converted by the parser.
  // -------------------------------------------------------------------------

  uint32_t hAcc;          ///< Horizontal position accuracy estimate in mm.
  uint32_t vAcc;          ///< Vertical   position accuracy estimate in mm.
  uint32_t sAcc;          ///< Ground speed accuracy estimate in mm/s.
  uint32_t headAcc;       ///< Heading accuracy estimate in 1e-5 degrees.
  uint16_t pDOP;          ///< Position dilution of precision, dimensionless × 100.

  // -------------------------------------------------------------------------
  // Fix status
  // -------------------------------------------------------------------------

  /// GNSS fix type using UBX convention:
  ///   0 = no fix,  1 = dead reckoning only,  2 = 2-D fix,
  ///   3 = 3-D fix, 4 = GNSS + dead reckoning combined.
  uint8_t  fixType;

  /// Validity and status flags.  See GNSS_FLAG_* constants in
  /// CommonParserConstants.hpp.
  uint8_t  validFlags;

  // -------------------------------------------------------------------------
  // UTC time
  //
  // Fields are zero-initialised until the relevant stage is implemented for
  // the active sensor family.  Check GNSS_FLAG_TIME_VALID and
  // GNSS_FLAG_DATE_VALID in validFlags before using.
  // -------------------------------------------------------------------------

  uint16_t year;          ///< UTC year, range 1999–2099.  0 = not yet valid.
  uint8_t  month;         ///< UTC month, range 1–12.
  uint8_t  day;           ///< UTC day of month, range 1–31.
  uint8_t  hour;          ///< UTC hour, range 0–23.
  uint8_t  minute;        ///< UTC minute, range 0–59.
  uint8_t  second;        ///< UTC second, range 0–60 (60 during a positive leap second).
  uint16_t millisecond;   ///< UTC sub-second in ms, range 0–999.
};
