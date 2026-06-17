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


/**
 * @enum GnssType
 * @brief Gnss selector for dualGNSS.
 */
enum class GnssType : uint8_t
{
    NONE   = 0U,
    CASIC  = 1U,
    UBX    = 2U
};


enum class GnssConfigStatus : uint8_t
{
    OK = 0U,                  ///< Configuration completed successfully.
    ERR_BAUD_NOT_FOUND,       ///< Module did not respond after baud-rate sweep.
    ERR_PROTO_FINAL_FAILED,   ///< Final protocol mask was rejected.
    ERR_RATE_FAILED,          ///< Navigation rate was rejected.
    ERR_MSG_FAILED,           ///< Message enable was rejected.
    ERR_SAVE_FAILED,          ///< Save to flash was rejected.
    ERR_VALIDATION_FAILED,    ///< Post-configuration readback did not match targets.
};

// ---------------------------------------------------------------------------
// UbxSeries
//
// Identifies the hardware generation of the connected u-blox GNSS module.
// Passed to the UbxGNSS constructor so that both the configurator and the
// parser can be directed to the correct command / message set without any
// run-time auto-detection.
//
// Generation mapping
// ------------------
//   UBX_M6_MINUS  — u-blox M6 and below
//                   Configurator: legacy CFG-PRT / CFG-RATE / CFG-MSG / CFG-CFG
//                   Parser:       NAV-POSLLH + NAV-SOL + NAV-VELNED epoch assembly
//
//   UBX_M7_M8     — u-blox M7 / M8  (protocol versions 15 – 23)
//                   Configurator: legacy CFG-PRT / CFG-RATE / CFG-MSG / CFG-CFG
//                   Parser:       NAV-PVT
//
//   UBX_M9_PLUS   — u-blox M9 / M10 (protocol versions >= 24)
//                   Configurator: CFG-VALSET / CFG-VALGET
//                   Parser:       NAV-PVT
//
//   UNKNOWN       — auto-detect via MON-VER during begin() only.
//                   Not valid for beginPassive() — an assertion fires at
//                   run-time if beginPassive() is called with UNKNOWN.
// ---------------------------------------------------------------------------

/**
 * @brief Hardware generation selector for u-blox GNSS modules.
 *
 * @details Pass the appropriate enumerator to the UbxGNSS constructor so the
 * library can select the correct configurator path and parser message set at
 * compile time rather than probing the module at run time.  UNKNOWN is the
 * only value that triggers run-time detection via MON-VER; it is not permitted
 * when calling UbxGNSS::beginPassive().
 */
enum class UbxSeries : uint8_t {
  UBX_M6_MINUS = 0U,  ///< u-blox M6 and below  — legacy CFG, triple-message epoch
  UBX_M7_M8    = 1U,  ///< u-blox M7 / M8       — legacy CFG, NAV-PVT
  UBX_M9_PLUS  = 2U,  ///< u-blox M9 / M10      — valset CFG, NAV-PVT
  UNKNOWN      = 3U   ///< Auto-detect in begin(); invalid for beginPassive()
};

/**
* @struct GnssConfigResult
* @brief  Unified configuration result returned by Gnss<>::configure().
*
* Common fields are populated regardless of module type.
* Protocol-specific fields are zeroed when the other module is selected;
* comments identify which fields belong to which protocol.
*/
struct GnssConfigResult
{
    // ------------------------------------------------------------------
    // Common fields — always populated
    // ------------------------------------------------------------------
    GnssConfigStatus status;           ///< Overall outcome of configuration.
    uint32_t         detectedBaud;     ///< TARGET_BAUD_RATE on success, 0 on failure.
    bool             validationPassed; ///< true if post-configuration readback passed.

    // ------------------------------------------------------------------
    // UBX-specific — zeroed when CASIC is selected
    // ------------------------------------------------------------------
    UbxSeries        detectedProvider; ///< Resolved hardware generation. UBX only.
    uint8_t          protocolVersion;  ///< MON-VER protocol version. UBX only.

    // ------------------------------------------------------------------
    // CASIC-specific — zeroed when UBX is selected
    // ------------------------------------------------------------------
    uint8_t          observedProtoMask;   ///< Protocol mask from CFG-PRT.  CASIC only.
    uint16_t         observedIntervalMs;  ///< Nav interval from CFG-RATE.  CASIC only.
    uint32_t         observedBaudRate;    ///< Baud rate from CFG-PRT.      CASIC only.
};