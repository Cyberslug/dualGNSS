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
// CasicConfigurator
//
// Configures a CASIC GNSS module over a HardwareSerial port to:
//
//   1. Accept CASIC binary and NMEA text input.
//   2. Output CASIC binary messages only.
//   3. Process at 100 ms intervals (10 Hz).
//   4. Emit NAV-PV messages once per navigation epoch.
//   5. Operate at TARGET_BAUD_RATE baud.
//
// CONFIGURATION SEQUENCE
// ----------------------
// Phase 0 — Blind baud-rate sweep (detectBaudRate)
//   Visits every baud-rate candidate in order and fires a CFG-PRT command
//   without waiting for acknowledgement.  The command sets all protocols on
//   all directions and requests a move to TARGET_BAUD_RATE.  It is sent
//   twice at each baud rate to catch the case where the first attempt
//   arrives while the module UART receive FIFO is full.
//
//   After the sweep the port is reopened at TARGET_BAUD_RATE and the
//   receive buffer is flushed.
//
// Phase 1 — Contact verification (verifyContact)
//   Polls CFG-PRT and waits for a valid response.  Success proves the
//   module received at least one command from the sweep and is now
//   communicating at TARGET_BAUD_RATE.  One retry is attempted.
//
// Phase 2 — Configuration
//   1. CFG-PRT  — final protocol mask (CASIC+NMEA in, CASIC out).
//   2. CFG-RATE — navigation update interval.
//   3. CFG-MSG  — enable NAV-PV output.
//   4. CFG-CFG  — save to flash.
//
// Phase 3 — Validation
//   Queries CFG-PRT and CFG-RATE and compares observed values against
//   targets.  Reports the observed values in CasicConfigResult for
//   diagnostic purposes.
//
//   Note: a CFG-PRT query causes the CASIC module to respond once per
//   UART port.  Phase 3 discards the first response from the non-active
//   port and parses the second active port).  If the module only returns one
//   response, the second readExpectedFrame call times out and validation
//   returns ERR_VALIDATION_FAILED even though the module is otherwise
//   correctly configured.  This has not been observed in practice but is
//   flagged for future reference
//
// UNRECOVERABLE STATES
// --------------------
// If the module has CASIC binary input disabled (NMEA-only input) the
// Phase 0 sweep can change the baud rate but not the IO configuration.
// Recovery requires a hardware reset or power cycle.  The only way to reach
// this state is through prior misconfiguration.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include "CasicConstants.hpp"
#include "CasicMessageBuilder.hpp"

// ---------------------------------------------------------------------------
// CasicConfigStatus — result codes returned by configure()
// ---------------------------------------------------------------------------

/**
 * @brief Result codes returned by CasicConfigurator::configure().
 */
enum class CasicConfigStatus : uint8_t {
  OK                  = 0U,  ///< Configuration completed successfully.
  ERR_BAUD_NOT_FOUND,        ///< Module did not respond after the baud-rate sweep.
  ERR_PROTO_FINAL_FAILED,    ///< CFG-PRT final protocol mask was rejected.
  ERR_RATE_FAILED,           ///< CFG-RATE navigation rate was rejected.
  ERR_MSG_FAILED,            ///< CFG-MSG NAV-PV enable was rejected.
  ERR_SAVE_FAILED,           ///< CFG-CFG save to flash was rejected.
  ERR_VALIDATION_FAILED,     ///< Post-configuration readback did not match targets.
};

// ---------------------------------------------------------------------------
// CasicConfigResult — diagnostic information returned by configure()
// ---------------------------------------------------------------------------

/**
 * @brief Diagnostic result returned by CasicConfigurator::configure().
 * @details On a successful return, status is OK, detectedBaud is TARGET_BAUD_RATE,
 *          validationPassed is true, and the observed* fields reflect the values
 *          read back from the module during Phase 3.
 */
struct CasicConfigResult {
  CasicConfigStatus status;            ///< Overall outcome of the configuration attempt.
  uint32_t          detectedBaud;      ///< TARGET_BAUD_RATE on success, 0 on failure.
  bool              validationPassed;  ///< true if Phase 3 validation passed.
  uint8_t           observedProtoMask; ///< Protocol mask read back from CFG-PRT in Phase 3.
  uint16_t          observedIntervalMs;///< Navigation interval read back from CFG-RATE in Phase 3.
  uint32_t          observedBaudRate;  ///< Baud rate read back from CFG-PRT in Phase 3.
};

// ---------------------------------------------------------------------------
// CasicConfigurator
// ---------------------------------------------------------------------------

class CasicConfigurator {
public:

  /**
   * @brief Runs the full Phase 0 / 1 / 2 / 3 configuration sequence.
   * @details Phase 0 (detectBaudRate): sweeps all candidate baud rates, sending a
   *          CFG-PRT command at each without waiting for ACKs.  Phase 1 (verifyContact):
   *          polls CFG-PRT to confirm the module is communicating at TARGET_BAUD_RATE.
   *          Phase 2: sends CFG-PRT, CFG-RATE, CFG-MSG, and CFG-CFG with ACK checks.
   *          Phase 3 (validateConfig): reads back CFG-PRT and CFG-RATE and compares
   *          against targets.  Do NOT call serial.begin() before calling configure().
   * @param serial Reference to the HardwareSerial port wired to the module.
   * @param rxPin  MCU pin connected to module TX (module → MCU).
   * @param txPin  MCU pin connected to module RX (MCU → module).
   * @return CasicConfigResult with status, detected baud rate, and observed configuration values.
   */
  CasicConfigResult configure(HardwareSerial& serial, int8_t rxPin, int8_t txPin);

  // Public constants used by external components (e.g. test helpers).
  static constexpr uint32_t TARGET_BAUD_RATE   = 115200UL;            ///< Target baud rate.
  static constexpr uint16_t TARGET_INTERVAL_MS = 100U;                ///< Target navigation interval (10 Hz).
  static constexpr uint8_t  TARGET_PROTO_MASK  = CASIC_PROTO_MASK_TARGET; ///< Target protocol mask.
  static constexpr uint16_t TARGET_NAV_PV_RATE = 1U;                  ///< NAV-PV output: every epoch.
  static constexpr uint32_t POST_CMD_DELAY_MS  = 50UL;                ///< Inter-command settling gap (ms).

private:

  int8_t m_rxPin; ///< MCU RX pin, stored from configure() for use by openSerial().
  int8_t m_txPin; ///< MCU TX pin, stored from configure() for use by openSerial().

  // Baud-rate candidates visited during the Phase 0 sweep (in order).
  static const uint32_t    BAUD_CANDIDATES[];
  static constexpr uint8_t NUM_BAUD_CANDIDATES = 6U;

  // Timing constants (ms).
  static constexpr uint32_t ACK_TIMEOUT_MS     = 500UL;
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
   * @brief Reads one complete, checksum-verified CASIC frame from the serial port.
   * @details Runs an inline state machine; frames larger than maxLen or with bad
   *          checksums are silently discarded and it continues until timeout.
   * @param serial    Reference to the open HardwareSerial port.
   * @param buf       Output buffer that receives the complete raw frame.
   * @param maxLen    Size of buf in bytes.
   * @param timeoutMs Maximum time to wait in milliseconds.
   * @return Total bytes in the received frame, or 0 on timeout.
   */
  static uint16_t readCasicFrame(HardwareSerial& serial, uint8_t* buf,
                                  uint16_t maxLen, uint32_t timeoutMs);

  /**
   * @brief Reads CASIC frames until one matching the expected class and ID is received.
   * @details Frames with non-matching class or ID are silently discarded; the search
   *          continues within the remaining timeout.
   * @param serial        Reference to the open HardwareSerial port.
   * @param expectedClass Expected message class byte (byte offset 4 in a CASIC frame).
   * @param expectedId    Expected message ID byte (byte offset 5).
   * @param buf           Output buffer that receives the matching frame.
   * @param maxLen        Size of buf in bytes.
   * @param timeoutMs     Maximum time to wait in milliseconds.
   * @return Total bytes in the matching frame, or 0 on timeout.
   */
  static uint16_t readExpectedFrame(HardwareSerial& serial,
                                     uint8_t expectedClass, uint8_t expectedId,
                                     uint8_t* buf, uint16_t maxLen,
                                     uint32_t timeoutMs);

  // -----------------------------------------------------------------------
  // Phase 0: blind baud-rate sweep
  // -----------------------------------------------------------------------

  /**
   * @brief Sweeps all baud-rate candidates, transmitting a CFG-PRT command at each.
   * @details Sends each command twice per candidate baud rate.  Reopens the port at
   *          TARGET_BAUD_RATE and flushes on return.
   * @param serial Reference to the HardwareSerial port.
   */
  void detectBaudRate(HardwareSerial& serial);

  // -----------------------------------------------------------------------
  // Phase 1: contact verification
  // -----------------------------------------------------------------------

  /**
   * @brief Polls CFG-PRT to verify the module is communicating at TARGET_BAUD_RATE.
   * @details A successful CFG-PRT response proves the Phase 0 sweep was effective.
   *          Two attempts are made; the second follows a fresh RX flush.
   * @param serial Reference to the open HardwareSerial port at TARGET_BAUD_RATE.
   * @return true if a CFG-PRT response was received, false on timeout.
   */
  bool verifyContact(HardwareSerial& serial);

  // -----------------------------------------------------------------------
  // Command execution helpers
  // -----------------------------------------------------------------------

  /**
   * @brief Writes a frame to the serial port and waits for an ACK or NACK.
   * @param serial    Reference to the open HardwareSerial port.
   * @param frame     Pointer to the frame bytes to transmit.
   * @param frameLen  Number of bytes to transmit.
   * @param expectCls Class byte of the sent command (for ACK matching).
   * @param expectId  ID byte of the sent command (for ACK matching).
   * @param timeoutMs Maximum time to wait for the ACK in milliseconds.
   * @return true if ACK-ACK was received, false on NACK or timeout.
   */
  static bool sendAndWaitAck(HardwareSerial& serial,
                              const uint8_t* frame, uint16_t frameLen,
                              uint8_t expectCls, uint8_t expectId,
                              uint32_t timeoutMs);

  /**
   * @brief Waits for a CASIC ACK or NACK for a specific command class and ID.
   * @details Non-ACK frames and ACKs for other commands are silently discarded.
   * @param serial    Reference to the open HardwareSerial port.
   * @param expectCls Class byte of the command awaiting acknowledgement.
   * @param expectId  ID byte of the command awaiting acknowledgement.
   * @param timeoutMs Maximum time to wait in milliseconds.
   * @param isNack    Output: set to true if the received response was a NACK.
   * @return true if ACK-ACK was received, false on NACK or timeout.
   */
  static bool waitForAck(HardwareSerial& serial,
                          uint8_t expectCls, uint8_t expectId,
                          uint32_t timeoutMs, bool& isNack);

  // -----------------------------------------------------------------------
  // Phase 2: configuration steps
  // -----------------------------------------------------------------------

  /**
   * @brief Sends CFG-PRT with the final target protocol mask and waits for ACK.
   * @param serial Reference to the open HardwareSerial port.
   * @return true on ACK-ACK, false on NACK or timeout.
   */
  bool cfgPrtFinalProtocol(HardwareSerial& serial);

  /**
   * @brief Sends CFG-RATE with the specified navigation update interval and waits for ACK.
   * @param serial     Reference to the open HardwareSerial port.
   * @param intervalMs Navigation update interval in milliseconds.
   * @return true on ACK-ACK, false on NACK or timeout.
   */
  bool cfgRate(HardwareSerial& serial, uint16_t intervalMs);

  /**
   * @brief Sends CFG-MSG to enable NAV-PV output at the specified rate and waits for ACK.
   * @param serial Reference to the open HardwareSerial port.
   * @param rate   Output rate: 0 = disabled, 1 = every epoch, N = every Nth epoch.
   * @return true on ACK-ACK, false on NACK or timeout.
   */
  bool cfgMsgNavPv(HardwareSerial& serial, uint16_t rate);

  /**
   * @brief Sends CFG-MSG to enable NAV-TIMEUTC output at the specified rate.
   */
  bool cfgMsgNavTimeUtc(HardwareSerial& serial, uint16_t rate);

  /**
   * @brief Sends CFG-CFG to save all configuration groups to flash and waits for ACK.
   * @param serial Reference to the open HardwareSerial port.
   * @return true on ACK-ACK, false on NACK or timeout.
   */
  bool cfgCfgSave(HardwareSerial& serial);

  // -----------------------------------------------------------------------
  // Phase 3: validation
  // -----------------------------------------------------------------------

  /**
   * @brief Queries CFG-PRT and CFG-RATE and compares observed values against targets.
   * @details Populates result.observedProtoMask, result.observedBaudRate, and
   *          result.observedIntervalMs with values read from the module.  The CFG-PRT
   *          query elicits one response per UART; the first, non-active, port is discarded 
   *          and the second, active, port is parsed.  If only one response
   *          arrives, the second readExpectedFrame call times out and this function
   *          returns false — see the class header note for details.
   * @param serial Reference to the open HardwareSerial port.
   * @param result Reference to the CasicConfigResult to populate with observed values.
   * @return true if all observed values match their targets, false otherwise.
   */
  static bool validateConfig(HardwareSerial& serial, CasicConfigResult& result);
};
