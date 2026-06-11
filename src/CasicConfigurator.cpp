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

#include "CasicConfigurator.hpp"
#include <string.h>

// ---------------------------------------------------------------------------
// Baud-rate candidates for the Phase 0 sweep.
// Most likely states first: factory default (115200), then other candidates
// in descending likelihood.
// ---------------------------------------------------------------------------
const uint32_t CasicConfigurator::BAUD_CANDIDATES[CasicConfigurator::NUM_BAUD_CANDIDATES] = {
  115200UL, 9600UL, 38400UL, 57600UL, 19200UL, 230400UL
};

// Receive buffer large enough for any CASIC frame processed here.
// Largest expected frame: CFG-PRT response (10 overhead + 8 payload = 18 bytes).
// 64 bytes provides ample margin.
static constexpr uint16_t RX_BUF_LEN = 64U;

// ===========================================================================
// Public entry point
// ===========================================================================

/**
 * @brief Runs the full Phase 0 / 1 / 2 / 3 configuration sequence.
 * @details Initialises all result fields to their failure-state defaults before
 *          starting so that any early return yields a defined, meaningful struct.
 *          Phase 2 steps are executed in a fixed order — CFG-PRT, CFG-RATE,
 *          CFG-MSG, CFG-CFG — each with its own error code so the caller can
 *          identify exactly which step failed.  Phase 3 populates the observed*
 *          fields of result regardless of whether the validation ultimately passes.
 */
CasicConfigResult CasicConfigurator::configure(HardwareSerial& serial,
                                               int8_t rxPin, int8_t txPin)
{
  m_rxPin = rxPin;
  m_txPin = txPin;

  CasicConfigResult result{};
  result.status             = CasicConfigStatus::ERR_BAUD_NOT_FOUND;
  result.detectedBaud       = 0UL;
  result.validationPassed   = false;
  result.observedProtoMask  = 0U;
  result.observedIntervalMs = 0U;
  result.observedBaudRate   = 0UL;

  // Phase 0: sweep all baud-rate candidates.
  //          Port is reopened at TARGET_BAUD_RATE on return.
  detectBaudRate(serial);

  // Phase 1: verify the module is communicating at TARGET_BAUD_RATE.
  if (!verifyContact(serial)) {
    return result;  // status remains ERR_BAUD_NOT_FOUND
  }
  result.detectedBaud = TARGET_BAUD_RATE;

  // Phase 2: apply configuration steps with ACK verification.
  if (!cfgPrtFinalProtocol(serial)) {
    result.status = CasicConfigStatus::ERR_PROTO_FINAL_FAILED;
    return result;
  }

  if (!cfgRate(serial, TARGET_INTERVAL_MS)) {
    result.status = CasicConfigStatus::ERR_RATE_FAILED;
    return result;
  }

  if (!cfgMsgNavPv(serial, TARGET_NAV_PV_RATE)) {
    result.status = CasicConfigStatus::ERR_MSG_FAILED;
    return result;
  }

  if (!cfgCfgSave(serial)) {
    result.status = CasicConfigStatus::ERR_SAVE_FAILED;
    return result;
  }

  // Phase 3: read back and validate.  Always populates result.observed* fields
  // so the caller can inspect what the module actually reported.
  result.validationPassed = validateConfig(serial, result);
  result.status = result.validationPassed
    ? CasicConfigStatus::OK
    : CasicConfigStatus::ERR_VALIDATION_FAILED;

  return result;
}

// ===========================================================================
// Serial helpers
// ===========================================================================

/**
 * @brief Closes and reopens the serial port at the specified baud rate.
 * @details Uses the four-argument ESP32 overload of HardwareSerial::begin().
 *          m_rxPin and m_txPin must be set before calling.
 */
void CasicConfigurator::openSerial(HardwareSerial& serial, uint32_t baud)
{
  serial.end();
  serial.begin(baud, SERIAL_8N1, m_rxPin, m_txPin);
}

/**
 * @brief Discards all pending receive bytes for approximately 50 ms.
 * @details The 50 ms window outlasts any in-flight CASIC or NMEA frame at any
 *          candidate baud rate so the receive buffer is clean for the next poll.
 */
void CasicConfigurator::flushRx(HardwareSerial& serial)
{
  const uint32_t deadline = millis() + 50U;
  while (millis() < deadline) {
    while (serial.available() > 0) {
      (void)serial.read();
    }
  }
}

/**
 * @brief Reads one complete, checksum-verified CASIC frame from the serial port.
 * @details Runs an inline CASIC frame state machine.  The frame structure is:
 *          0xBA 0xCE [lenL lenH] [cls] [id] [payload...] [ck0..ck3].
 *          Frames whose declared payload length would exceed maxLen are discarded.
 *          The 32-bit checksum is seeded from id, class, and length when the ID
 *          byte arrives, then accumulated word-by-word as the payload is received.
 *          A frame with a mismatched checksum is discarded and the search continues.
 */
uint16_t CasicConfigurator::readCasicFrame(HardwareSerial& serial,
                                            uint8_t* buf, uint16_t maxLen,
                                            uint32_t timeoutMs)
{
  enum State : uint8_t {
    S_SYNC1, S_SYNC2, S_LEN_L, S_LEN_H,
    S_CLASS, S_ID, S_PAYLOAD,
    S_CK_0, S_CK_1, S_CK_2, S_CK_3
  };

  State    state      = S_SYNC1;
  uint16_t pos        = 0U;
  uint16_t payloadLen = 0U;
  uint16_t payloadIdx = 0U;
  uint8_t  cls        = 0U;
  uint8_t  id         = 0U;
  uint32_t ckAccum    = 0U;
  uint8_t  ckBuf[4]   = {0U, 0U, 0U, 0U};

  // Payload accumulation buffer — points into buf[] at the payload start offset.
  // The CASIC frame header occupies 6 bytes (sync×2, len×2, cls, id).
  static constexpr uint16_t HEADER_LEN = 6U;

  const uint32_t deadline = millis() + timeoutMs;

  while (millis() < deadline) {
    if (serial.available() <= 0) { continue; }
    const uint8_t b = static_cast<uint8_t>(serial.read());

    switch (state) {
      case S_SYNC1:
        if (b == CASIC_SYNC1) {
          pos = 0U;
          buf[pos++] = b;
          state = S_SYNC2;
        }
        break;

      case S_SYNC2:
        if (b == CASIC_SYNC2) {
          buf[pos++] = b;
          state = S_LEN_L;
        } else {
          // A second 0xBA may be the start of a real frame.
          state = (b == CASIC_SYNC1) ? S_SYNC2 : S_SYNC1;
          pos = 0U;
        }
        break;

      case S_LEN_L:
        buf[pos++] = b;
        payloadLen = static_cast<uint16_t>(b);
        state = S_LEN_H;
        break;

      case S_LEN_H:
        buf[pos++] = b;
        payloadLen |= static_cast<uint16_t>(static_cast<uint32_t>(b) << 8U);
        if (static_cast<uint16_t>(payloadLen + CASIC_FRAME_OVERHEAD) > maxLen) {
          // Frame too large for the buffer — discard and resync.
          state = S_SYNC1; pos = 0U;
        } else {
          state = S_CLASS;
        }
        break;

      case S_CLASS:
        buf[pos++] = b;
        cls = b;
        state = S_ID;
        break;

      case S_ID:
        buf[pos++] = b;
        id = b;
        // Seed the 32-bit checksum.  Non-overlapping fields: id[31:24],
        // cls[23:16], payloadLen[15:0].  | and + are equivalent here but
        // | better expresses the intent.
        ckAccum  = (static_cast<uint32_t>(id)  << 24U)
                 | (static_cast<uint32_t>(cls)  << 16U)
                 |  static_cast<uint32_t>(payloadLen);
        payloadIdx = 0U;
        if (payloadLen == 0U) {
          state = S_CK_0;
        } else {
          state = S_PAYLOAD;
        }
        break;

      case S_PAYLOAD:
        buf[pos++] = b;
        payloadIdx++;
        // Accumulate a complete 32-bit LE word once every 4 bytes.
        if ((payloadIdx % 4U) == 0U) {
          uint32_t word = 0U;
          memcpy(&word, &buf[HEADER_LEN + payloadIdx - 4U], sizeof(word));
          ckAccum += word;
        }
        if (payloadIdx == payloadLen) { state = S_CK_0; }
        break;

      case S_CK_0:
        ckBuf[0] = b; buf[pos++] = b; state = S_CK_1;
        break;

      case S_CK_1:
        ckBuf[1] = b; buf[pos++] = b; state = S_CK_2;
        break;

      case S_CK_2:
        ckBuf[2] = b; buf[pos++] = b; state = S_CK_3;
        break;

      case S_CK_3: {
        ckBuf[3] = b; buf[pos++] = b;
        // Reassemble the received 32-bit LE checksum.
        const uint32_t rxCk =
            static_cast<uint32_t>(ckBuf[0])
          | (static_cast<uint32_t>(ckBuf[1]) <<  8U)
          | (static_cast<uint32_t>(ckBuf[2]) << 16U)
          | (static_cast<uint32_t>(ckBuf[3]) << 24U);

        if (rxCk == ckAccum) { return pos; }
        // Checksum mismatch — discard and resync.
        state = S_SYNC1; pos = 0U;
        break;
      }

      default:
        state = S_SYNC1; pos = 0U;
        break;
    }
  }
  return 0U;
}

/**
 * @brief Reads CASIC frames, discarding any that do not match the expected class and ID.
 * @details In a CASIC frame the class is at byte offset 4 and the ID at offset 5.
 *          Non-matching frames consume some of the remaining timeout budget but are
 *          otherwise harmless.  Returns as soon as a matching frame is found or the
 *          deadline expires.
 */
uint16_t CasicConfigurator::readExpectedFrame(HardwareSerial& serial,
                                               uint8_t expectedClass, uint8_t expectedId,
                                               uint8_t* buf, uint16_t maxLen,
                                               uint32_t timeoutMs)
{
  const uint32_t deadline = millis() + timeoutMs;

  while (true) {
    const uint32_t now = millis();
    if (now >= deadline) { return 0U; }

    const uint16_t rxLen = readCasicFrame(serial, buf, maxLen, deadline - now);
    if (rxLen == 0U) { return 0U; }

    // CASIC frame byte layout: [sync×2][len×2][cls][id][payload...][ck×4]
    if ((buf[4] == expectedClass) && (buf[5] == expectedId)) { return rxLen; }
    // Valid frame but wrong type — keep waiting within remaining budget.
  }
}

// ===========================================================================
// Phase 0: blind baud-rate sweep
// ===========================================================================

/**
 * @brief Sends one open-channel CFG-PRT command that enables all protocols and requests
 *        TARGET_BAUD_RATE.
 * @details This is a file-scope helper called by detectBaudRate(); it is not a class
 *          member.  No acknowledgement is expected or awaited.  CASIC has no NMEA-based
 *          recovery equivalent to UBX's PUBX,41 sentence: if the module has CASIC
 *          binary input disabled then this command cannot reach it and the sweep
 *          will have no effect.  Recovery in that case requires a hardware reset.
 *          CASIC_PROTO_MASK_ALL is used during the sweep so that any prior
 *          misconfiguration of protocol masks does not block reception.
 */
static void sendOpenBurst(HardwareSerial& serial)
{
  uint8_t frame[CASIC_MAX_FRAME_BUF];

  const uint16_t len = CasicMessageBuilder::buildCfgPrt(
    CASIC_PORT_CURRENT,
    CASIC_PROTO_MASK_ALL,
    CASIC_UART_MODE_8N1,
    CasicConfigurator::TARGET_BAUD_RATE,
    frame, static_cast<uint16_t>(sizeof(frame)));

  if (len > 0U) {
    serial.write(frame, len);
    serial.flush();
  }
}

/**
 * @brief Iterates over BAUD_CANDIDATES, transmitting a double open-channel burst at each.
 * @details The burst is sent twice at each baud rate because the first attempt may
 *          arrive while the module UART receive FIFO is full following a previous
 *          command.  The port is reopened at TARGET_BAUD_RATE and flushed on return
 *          so Phase 1 can begin polling immediately.
 */
void CasicConfigurator::detectBaudRate(HardwareSerial& serial)
{
  for (uint8_t i = 0U; i < NUM_BAUD_CANDIDATES; ++i) {
    openSerial(serial, BAUD_CANDIDATES[i]);
    delay(BAUD_OPEN_DELAY_MS);

    // Send the burst twice.  The first attempt may arrive while the module
    // UART receive FIFO is full; the second catches that case.
    sendOpenBurst(serial);
    delay(SWEEP_SETTLE_MS);
    sendOpenBurst(serial);
    delay(SWEEP_SETTLE_MS);
  }

  // Reopen at the target baud rate and discard residual output.
  openSerial(serial, TARGET_BAUD_RATE);
  flushRx(serial);
}

// ===========================================================================
// Phase 1: contact verification
// ===========================================================================

/**
 * @brief Polls CFG-PRT to confirm the module is responding at TARGET_BAUD_RATE.
 * @details A zero-payload CFG-PRT query is used as the probe — it is guaranteed to
 *          elicit at least one response if the sweep succeeded.  Two attempts are
 *          made; the second follows a fresh flush in case residual output from the
 *          sweep is blocking the buffer.  Any valid CFG-PRT response is sufficient
 *          to confirm contact; the payload content is not checked at this stage.
 */
bool CasicConfigurator::verifyContact(HardwareSerial& serial)
{
  uint8_t  txFrame[CASIC_MAX_FRAME_BUF];
  uint8_t  rxBuf[RX_BUF_LEN];

  const uint16_t queryLen = CasicMessageBuilder::buildQuery(
    CASIC_CLASS_CFG, CASIC_ID_CFG_PRT,
    txFrame, static_cast<uint16_t>(sizeof(txFrame)));

  for (uint8_t attempt = 0U; attempt < 2U; ++attempt) {
    flushRx(serial);
    serial.write(txFrame, queryLen);
    serial.flush();

    // Accept any CFG-PRT response — content validated in Phase 3.
    const uint16_t rxLen = readExpectedFrame(
      serial, CASIC_CLASS_CFG, CASIC_ID_CFG_PRT,
      rxBuf, static_cast<uint16_t>(sizeof(rxBuf)),
      ACK_TIMEOUT_MS);

    if (rxLen > 0U) { return true; }
  }
  return false;
}

// ===========================================================================
// Command execution helpers
// ===========================================================================

/**
 * @brief Transmits a pre-built frame and waits for the matching ACK or NACK.
 * @details Delegates to waitForAck() after writing; isNack is not propagated
 *          to the caller since all Phase 2 helpers treat NACK and timeout
 *          identically (both are failures).
 */
bool CasicConfigurator::sendAndWaitAck(HardwareSerial& serial,
                                        const uint8_t* frame, uint16_t frameLen,
                                        uint8_t expectCls, uint8_t expectId,
                                        uint32_t timeoutMs)
{
  serial.write(frame, frameLen);
  serial.flush();
  bool isNack = false;
  return waitForAck(serial, expectCls, expectId, timeoutMs, isNack);
}

/**
 * @brief Waits for a CASIC ACK or NACK matching the specified command class and ID.
 * @details CASIC ACK payload: U1 clsID, U1 msgID, U2 reserved.  In a raw frame
 *          the class is at buf[4], id at buf[5], and the payload begins at buf[6].
 *          Non-ACK frames and ACKs for other commands are silently discarded.
 *          isNack is set to true only when a NACK for the expected command arrives;
 *          it is left false on timeout so the caller can distinguish the two cases.
 */
bool CasicConfigurator::waitForAck(HardwareSerial& serial,
                                    uint8_t expectCls, uint8_t expectId,
                                    uint32_t timeoutMs, bool& isNack)
{
  uint8_t buf[RX_BUF_LEN];
  const uint32_t deadline = millis() + timeoutMs;
  isNack = false;

  while (true) {
    const uint32_t now = millis();
    if (now >= deadline) { return false; }

    const uint16_t rxLen = readCasicFrame(serial, buf,
                                           static_cast<uint16_t>(sizeof(buf)),
                                           deadline - now);
    // Minimum ACK frame: 10 bytes (6 overhead + 4 payload).
    if (rxLen < 10U) { continue; }
    if (buf[4] != CASIC_CLASS_ACK) { continue; }

    // Check whether this ACK/NACK is for the command we sent.
    const bool forUs = (buf[6] == expectCls) && (buf[7] == expectId);

    if (buf[5] == CASIC_ID_ACK_NACK) {
      if (forUs) { isNack = true; return false; }
      continue;
    }
    if (buf[5] != CASIC_ID_ACK_ACK) { continue; }
    if (forUs) { return true; }
    // ACK for a different command — discard and keep waiting.
  }
}

// ===========================================================================
// Phase 2: configuration steps
// ===========================================================================

/**
 * @brief Builds and sends a CFG-PRT command with the final target protocol mask.
 * @details Uses TARGET_PROTO_MASK (CASIC+NMEA in, CASIC out) and TARGET_BAUD_RATE.
 *          The Phase 0 sweep may have left all protocols open; this step narrows
 *          the output to CASIC binary only, which is required by the parser.
 */
bool CasicConfigurator::cfgPrtFinalProtocol(HardwareSerial& serial)
{
  uint8_t frame[CASIC_MAX_FRAME_BUF];

  const uint16_t len = CasicMessageBuilder::buildCfgPrt(
    CASIC_PORT_CURRENT,
    TARGET_PROTO_MASK,
    CASIC_UART_MODE_8N1,
    TARGET_BAUD_RATE,
    frame, static_cast<uint16_t>(sizeof(frame)));

  const bool ok = sendAndWaitAck(serial, frame, len,
                                  CASIC_CLASS_CFG, CASIC_ID_CFG_PRT,
                                  ACK_TIMEOUT_MS);
  if (ok) { delay(POST_CMD_DELAY_MS); }
  return ok;
}

/**
 * @brief Builds and sends a CFG-RATE command with the specified navigation interval.
 * @details intervalMs is taken directly from the caller; no fallback is attempted
 *          here.  If the module rejects TARGET_INTERVAL_MS the caller (configure())
 *          maps the failure to ERR_RATE_FAILED without retrying.
 */
bool CasicConfigurator::cfgRate(HardwareSerial& serial, uint16_t intervalMs)
{
  uint8_t frame[CASIC_MAX_FRAME_BUF];

  const uint16_t len = CasicMessageBuilder::buildCfgRate(
    intervalMs,
    frame, static_cast<uint16_t>(sizeof(frame)));

  const bool ok = sendAndWaitAck(serial, frame, len,
                                  CASIC_CLASS_CFG, CASIC_ID_CFG_RATE,
                                  ACK_TIMEOUT_MS);
  if (ok) { delay(POST_CMD_DELAY_MS); }
  return ok;
}

/**
 * @brief Builds and sends a CFG-MSG command to enable NAV-PV output.
 * @details rate is typically TARGET_NAV_PV_RATE (1), meaning one NAV-PV message
 *          per navigation epoch.  A rate of 0 would disable output.
 */
bool CasicConfigurator::cfgMsgNavPv(HardwareSerial& serial, uint16_t rate)
{
  uint8_t frame[CASIC_MAX_FRAME_BUF];

  const uint16_t len = CasicMessageBuilder::buildCfgMsg(
    CASIC_CLASS_NAV, CASIC_ID_NAV_PV,
    rate,
    frame, static_cast<uint16_t>(sizeof(frame)));

  const bool ok = sendAndWaitAck(serial, frame, len,
                                  CASIC_CLASS_CFG, CASIC_ID_CFG_MSG,
                                  ACK_TIMEOUT_MS);
  if (ok) { delay(POST_CMD_DELAY_MS); }
  return ok;
}

/**
 * @brief Builds and sends a CFG-CFG command to save all configuration groups to flash.
 * @details Uses CASIC_CFG_SAVE_MASK (all groups) and CASIC_CFG_MODE_SAVE.
 *          This is the final Phase 2 step; the saved configuration survives power cycling.
 */
bool CasicConfigurator::cfgCfgSave(HardwareSerial& serial)
{
  uint8_t frame[CASIC_MAX_FRAME_BUF];

  const uint16_t len = CasicMessageBuilder::buildCfgCfg(
    CASIC_CFG_SAVE_MASK, CASIC_CFG_MODE_SAVE,
    frame, static_cast<uint16_t>(sizeof(frame)));

  const bool ok = sendAndWaitAck(serial, frame, len,
                                  CASIC_CLASS_CFG, CASIC_ID_CFG_CFG,
                                  ACK_TIMEOUT_MS);
  if (ok) { delay(POST_CMD_DELAY_MS); }
  return ok;
}

// ===========================================================================
// Phase 3: validation
// ===========================================================================

/**
 * @brief Polls CFG-PRT and CFG-RATE, compares observed values against targets.
 * @details CFG-PRT query: the module responds once per UART port.  The first
 *          response non-active port is silently discarded; the second active
 *          port is parsed.  If only one response arrives the second
 *          readExpectedFrame() call times out and false is returned — see the
 *          class-level header note for the accepted-limitation context.
 *          All three observed fields are populated (or left at 0 on timeout)
 *          before the function returns, so the caller always has diagnostic data.
 */
bool CasicConfigurator::validateConfig(HardwareSerial& serial,
                                        CasicConfigResult& result)
{
  uint8_t  txFrame[CASIC_MAX_FRAME_BUF];
  uint8_t  rxBuf[RX_BUF_LEN];

  // -------------------------------------------------------------------------
  // CFG-PRT readback
  // -------------------------------------------------------------------------
  const uint16_t queryLen = CasicMessageBuilder::buildQuery(
    CASIC_CLASS_CFG, CASIC_ID_CFG_PRT,
    txFrame, static_cast<uint16_t>(sizeof(txFrame)));

  flushRx(serial);
  serial.write(txFrame, queryLen);
  serial.flush();

  // Discard the first non-active port response; the active port is last.
  (void)readExpectedFrame(serial, CASIC_CLASS_CFG, CASIC_ID_CFG_PRT,
                           rxBuf, static_cast<uint16_t>(sizeof(rxBuf)),
                           ACK_TIMEOUT_MS);

  // Parse the second response, the active port.
  const uint16_t prtLen = readExpectedFrame(
    serial, CASIC_CLASS_CFG, CASIC_ID_CFG_PRT,
    rxBuf, static_cast<uint16_t>(sizeof(rxBuf)),
    ACK_TIMEOUT_MS);

  if (prtLen >= static_cast<uint16_t>(CASIC_FRAME_OVERHEAD + CASIC_CFG_PRT_PAYLOAD_LEN)) {
    // CFG-PRT payload begins at offset 6.
    // Layout: U1 portId, U1 protoMask, U2 uartMode, U4 baudRate.
    CasicMessageBuilder::parseCfgPrtResponse(rxBuf,
                                              result.observedProtoMask,
                                              result.observedBaudRate);
  }

  // -------------------------------------------------------------------------
  // CFG-RATE readback
  // -------------------------------------------------------------------------
  const uint16_t rateQueryLen = CasicMessageBuilder::buildQuery(
    CASIC_CLASS_CFG, CASIC_ID_CFG_RATE,
    txFrame, static_cast<uint16_t>(sizeof(txFrame)));

  flushRx(serial);
  serial.write(txFrame, rateQueryLen);
  serial.flush();

  const uint16_t rateLen = readExpectedFrame(
    serial, CASIC_CLASS_CFG, CASIC_ID_CFG_RATE,
    rxBuf, static_cast<uint16_t>(sizeof(rxBuf)),
    ACK_TIMEOUT_MS);

  if (rateLen >= static_cast<uint16_t>(CASIC_FRAME_OVERHEAD + CASIC_CFG_RATE_PAYLOAD_LEN)) {
    result.observedIntervalMs = CasicMessageBuilder::parseCfgRateResponse(rxBuf);
  }

  // -------------------------------------------------------------------------
  // Validation: compare observed values against targets.
  // -------------------------------------------------------------------------
  const bool baudOk  = (result.observedBaudRate   == TARGET_BAUD_RATE);
  const bool protoOk = (result.observedProtoMask  == TARGET_PROTO_MASK);
  const bool rateOk  = (result.observedIntervalMs == TARGET_INTERVAL_MS);

  return baudOk && protoOk && rateOk;
}
