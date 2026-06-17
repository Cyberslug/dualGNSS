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

// ---------------------------------------------------------------------------
// UbxConfigurator
//
// Configures a u-blox GNSS module (M6 through M10) over a HardwareSerial
// port to:
//
//   1. Accept UBX and NMEA input; output UBX binary only.
//   2. Process at 100 ms intervals (10 Hz).
//   3. Emit the correct navigation message:
//        NAV-PVT              — M7/M8/M9/M10 (GpsProvider::UBX_M7_M8 or UBX_M9_PLUS)
//        NAV-POSLLH + NAV-SOL + NAV-VELNED  — M6 (GpsProvider::UBX_M6_MINUS)
//   4. Operate at TARGET_BAUD_RATE baud.
//   5. Persist all settings to all available non-volatile storage.
//
// CONFIGURATION SEQUENCE
// ----------------------
// Phase 0 — Blind baud-rate sweep (detectBaudRate)
//   Visits every baud-rate candidate and fires a burst of three commands
//   without waiting for acknowledgement.  The burst is repeated once per
//   baud rate to catch the case where the first attempt arrives while the
//   module UART receive FIFO is full.  Commands sent at each baud rate:
//
//     a) PUBX,41  — NMEA sentence, accepted regardless of UBX-input state.
//                   Re-enables UBX binary input so commands (b) and (c) land.
//     b) CFG-VALSET (RAM layer) — M9/M10 path; sets IO protocol + TARGET_BAUD_RATE baud.
//     c) CFG-PRT               — M6/M7/M8 path; same settings.
//
//   After the sweep the port is reopened at TARGET_BAUD_RATE baud and the RX buffer
//   is flushed.  At least one command in the sweep will have reached the
//   module regardless of its prior baud rate or IO-protocol state.
//
// Phase 1 — Contact verification and module identification (identifyModule)
//   Polls UBX-MON-VER at TARGET_BAUD_RATE baud.  Proves the sweep landed.  Also
//   parses the protocol version to resolve the generation when UNKNOWN was
//   passed to configure():
//
//     Protocol <= 14       →  UBX_M6_MINUS (legacy CFG, triple-message epoch)
//     14 < Protocol < 27   →  UBX_M7_M8    (legacy CFG, NAV-PVT)
//     Protocol >= 27       →  UBX_M9_PLUS  (CFG-VALSET, NAV-PVT)
//
//   When a specific generation is passed to configure(), the protocol version
//   is still read (for diagnostics) but does not override the caller's value.
//
// Phase 2 — Configuration (configureLegacy / configureValset)
//   Port is already at TARGET_BAUD_RATE baud with UBX binary accepted.  No baud-rate
//   change is required or performed here.
//
// EDGE CASE
// ---------
// If the module has ALL input protocols disabled the sweep cannot reach it.
// Recovery requires a hardware reset or power cycle.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include "GnssTypes.hpp"
#include "UbxConstants.hpp"
#include "UbxMessageBuilder.hpp"

// ---------------------------------------------------------------------------
// UbxConfigResult — diagnostic information returned by configure()
// ---------------------------------------------------------------------------

/**
 * @brief Diagnostic result returned by UbxConfigurator::configure().
 * @details On a successful return, status is OK, detectedProvider is a specific
 *          generation value (never UNKNOWN), detectedBaud is TARGET_BAUD_RATE,
 *          and validationPassed is true.
 */
struct UbxConfigResult {
  GnssConfigStatus status;           ///< Overall outcome of the configuration attempt.
  UbxSeries        detectedProvider; ///< Resolved hardware generation; never UNKNOWN on return.
  uint32_t         detectedBaud;     ///< TARGET_BAUD_RATE on success, 0 on failure.
  uint8_t          protocolVersion;  ///< Protocol version from MON-VER; 0 if not parsed.
  bool             validationPassed; ///< true if the post-configuration readback succeeded.
};

// ---------------------------------------------------------------------------
// UbxConfigurator
// ---------------------------------------------------------------------------

class UbxConfigurator {
public:

  /**
   * @brief Runs the full Phase 0 / 1 / 2 configuration sequence.
   * @details Phase 0 (detectBaudRate): visits every candidate baud rate and fires
   *          a PUBX,41 + CFG-VALSET + CFG-PRT burst without waiting for ACKs, then
   *          reopens at TARGET_BAUD_RATE.  Phase 1 (identifyModule): polls MON-VER
   *          to confirm contact and identify the module generation.  Phase 2
   *          (configureLegacy or configureValset): applies final IO protocol,
   *          navigation rate, and message enables; saves to non-volatile storage.
   *          Do NOT call serial.begin() before calling configure(); the configurator
   *          manages the baud rate internally throughout.
   * @param serial     HardwareSerial port wired to the module.
   * @param rxPin      MCU pin connected to module TX (module → MCU).
   * @param txPin      MCU pin connected to module RX (MCU → module).
   * @param generation Hardware generation hint; UNKNOWN triggers MON-VER auto-detection.
   * @return UbxConfigResult with status, detected generation, baud rate, and protocol
   *         version.  detectedProvider is never UNKNOWN on return.
   */
  UbxConfigResult configure(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                             UbxSeries generation = UbxSeries::UNKNOWN);

  // Public constants used by external components (e.g. test helpers).
  static constexpr uint32_t TARGET_BAUD_RATE    = 115200UL; ///< Target baud rate after configuration.
  static constexpr uint16_t TARGET_MEAS_RATE_MS = 100U;     ///< Preferred measurement rate (10 Hz).
  static constexpr uint16_t FALLBACK_RATE_MS    = 1000U;    ///< Fallback measurement rate (1 Hz).
  static constexpr uint32_t POST_CMD_DELAY_MS   = 50UL;     ///< Inter-command settling gap (ms).

private:

  int8_t      m_rxPin;           ///< MCU RX pin, stored from configure() for use by openSerial().
  int8_t      m_txPin;           ///< MCU TX pin, stored from configure() for use by openSerial().
  uint8_t     m_protocolVersion; ///< UBX protocol version read from MON-VER in Phase 1.
  UbxSeries m_generation;      ///< Resolved generation used throughout the configure() call.

  // Baud-rate candidates visited during the Phase 0 sweep (in order).
  static const uint32_t    BAUD_CANDIDATES[];
  static constexpr uint8_t NUM_BAUD_CANDIDATES = 6U;

  // Timing constants (ms).
  static constexpr uint32_t ACK_TIMEOUT_MS     = 500UL;  // RAM writes
  static constexpr uint32_t SAVE_TIMEOUT_MS    = 2000UL; // Flash/BBR commit
  static constexpr uint32_t MONVER_TIMEOUT_MS  = 1500UL;
  static constexpr uint32_t SWEEP_SETTLE_MS    = 150UL;  // between burst pairs
  static constexpr uint32_t BAUD_OPEN_DELAY_MS =  20UL;  // after serial.begin()

  // -----------------------------------------------------------------------
  // Serial helpers
  // -----------------------------------------------------------------------

  /**
   * @brief Closes and reopens the serial port at the specified baud rate.
   * @param serial Reference to the HardwareSerial port to reconfigure.
   * @param baud   New baud rate in bits/s.
   */
  void openSerial(HardwareSerial& serial, uint32_t baud);

  /**
   * @brief Discards all pending receive bytes for approximately 50 ms.
   * @param serial Reference to the HardwareSerial port to drain.
   */
  static void flushRx(HardwareSerial& serial);

  /**
   * @brief Reads one complete, checksum-verified UBX frame from the serial port.
   * @details Runs an inline state machine; frames larger than maxLen or with bad
   *          checksums are silently discarded and hunting continues until timeout.
   * @param serial    Reference to the open HardwareSerial port.
   * @param buf       Output buffer that receives the complete raw frame.
   * @param maxLen    Size of buf in bytes.
   * @param timeoutMs Maximum time to wait in milliseconds.
   * @return Total bytes in the received frame, or 0 on timeout.
   */
  static uint16_t readUbxFrame(HardwareSerial& serial, uint8_t* buf,
                               uint16_t maxLen, uint32_t timeoutMs);

  /**
   * @brief Reads UBX frames until one matching the expected class and ID is received.
   * @details Frames with non-matching class or ID are silently discarded; the search
   *          continues within the remaining timeout.
   * @param serial    Reference to the open HardwareSerial port.
   * @param expectCls Expected message class byte.
   * @param expectId  Expected message ID byte.
   * @param buf       Output buffer that receives the matching frame.
   * @param maxLen    Size of buf in bytes.
   * @param timeoutMs Maximum time to wait in milliseconds.
   * @return Total bytes in the matching frame, or 0 on timeout.
   */
  static uint16_t readExpectedFrame(HardwareSerial& serial,
                                    uint8_t expectCls, uint8_t expectId,
                                    uint8_t* buf, uint16_t maxLen,
                                    uint32_t timeoutMs);

  /**
   * @brief Waits for a UBX-ACK-ACK or UBX-ACK-NAK for a specific command.
   * @details Non-ACK frames and ACKs for other commands are silently discarded.
   * @param serial    Reference to the open HardwareSerial port.
   * @param expectCls Class byte of the command awaiting acknowledgement.
   * @param expectId  ID byte of the command awaiting acknowledgement.
   * @param timeoutMs Maximum time to wait in milliseconds.
   * @return true if ACK-ACK was received, false on NAK or timeout.
   */
  static bool waitForAck(HardwareSerial& serial,
                         uint8_t expectCls, uint8_t expectId,
                         uint32_t timeoutMs);

  // -----------------------------------------------------------------------
  // Phase 0: blind baud-rate sweep
  // -----------------------------------------------------------------------

  /**
   * @brief Sweeps all baud-rate candidates, transmitting open-channel commands at each.
   * @details Sends a PUBX,41 + CFG-VALSET + CFG-PRT burst twice at every candidate
   *          baud rate.  Reopens the port at TARGET_BAUD_RATE and flushes on return.
   * @param serial Reference to the HardwareSerial port.
   */
  void detectBaudRate(HardwareSerial& serial);

  // -----------------------------------------------------------------------
  // Phase 1: contact verification and module identification
  // -----------------------------------------------------------------------

  /**
   * @brief Polls MON-VER to verify contact and read the module's protocol version.
   * @details Two attempts are made; the second follows a fresh RX flush.  Populates
   *          m_protocolVersion on success and resolves m_generation from it when
   *          UNKNOWN was passed to configure().
   * @param serial Reference to the open HardwareSerial port at TARGET_BAUD_RATE.
   * @return true if MON-VER was received and parsed successfully, false on timeout.
   */
  bool identifyModule(HardwareSerial& serial);

  /**
   * @brief Extracts the major protocol version number from a raw MON-VER frame buffer.
   * @details Scans the variable-length extension strings for one beginning with
   *          "PROTVER=" and parses the leading decimal integer.
   * @param monverBuf Pointer to the complete MON-VER frame including UBX header bytes.
   * @param bufLen    Total length of the buffer in bytes.
   * @return Major protocol version, or 0 if the PROTVER extension was not found.
   */
  static uint8_t parseProtocolVersion(const uint8_t* monverBuf, uint16_t bufLen);

  // -----------------------------------------------------------------------
  // Phase 2: configuration paths
  // -----------------------------------------------------------------------

  /**
   * @brief Configures M6 / M7 / M8 modules using legacy CFG commands.
   * @details Sends CFG-PRT (IO protocol and baud rate), CFG-RATE (10 Hz with 1 Hz
   *          fallback), CFG-MSG for the message set appropriate to m_generation,
   *          then CFG-CFG to save all settings to non-volatile storage.
   *          Calls validateLegacy() to confirm success.
   * @param serial Reference to the open HardwareSerial port.
   * @return GnssConfigStatus::OK on success, or an error code identifying the failing step.
   */
  GnssConfigStatus configureLegacy(HardwareSerial& serial);

  /**
   * @brief Configures M9 / M10 modules using a single CFG-VALSET command.
   * @details Writes IO protocol enables, navigation rate, and NAV-PVT output rate to
   *          all storage layers (RAM + BBR + Flash) in one atomic transaction.
   *          Calls validateValset() to confirm success.
   * @param serial Reference to the open HardwareSerial port.
   * @return GnssConfigStatus::OK on success, or an error code identifying the failing step.
   */
  GnssConfigStatus configureValset(HardwareSerial& serial);

  /**
   * @brief Reads back CFG-PRT and CFG-RATE to verify legacy configuration targets.
   * @details Checks that the output protocol is UBX-only and the baud rate is
   *          TARGET_BAUD_RATE.  Accepts both TARGET_MEAS_RATE_MS and FALLBACK_RATE_MS
   *          as valid navigation rates, since configureLegacy() may fall back to 1 Hz.
   * @param serial Reference to the open HardwareSerial port.
   * @return GnssConfigStatus::OK if targets match, ERR_VALIDATION_FAILED otherwise.
   */
  static GnssConfigStatus validateLegacy(HardwareSerial& serial);

  /**
   * @brief Reads back key configuration values to verify the valset configuration.
   * @details Queries UBXKEY_UART1_BAUDRATE, UBXKEY_UART1OUTPROT_UBX, and
   *          UBXKEY_RATE_MEAS from RAM via CFG-VALGET and compares each against its
   *          target.  Accepts TARGET_MEAS_RATE_MS or FALLBACK_RATE_MS for the rate.
   * @param serial Reference to the open HardwareSerial port.
   * @return GnssConfigStatus::OK if all keys match, ERR_VALIDATION_FAILED otherwise.
   */
  static GnssConfigStatus validateValset(HardwareSerial& serial);
};
