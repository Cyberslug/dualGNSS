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
// GpsProvider
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
enum class GpsProvider : uint8_t {
  UBX_M6_MINUS = 0U,  ///< u-blox M6 and below  — legacy CFG, triple-message epoch
  UBX_M7_M8    = 1U,  ///< u-blox M7 / M8       — legacy CFG, NAV-PVT
  UBX_M9_PLUS  = 2U,  ///< u-blox M9 / M10      — valset CFG, NAV-PVT
  UNKNOWN      = 3U   ///< Auto-detect in begin(); invalid for beginPassive()
};
