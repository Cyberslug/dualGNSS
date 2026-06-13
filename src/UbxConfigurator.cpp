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

#include "UbxConfigurator.hpp"
#include <string.h>

// ---------------------------------------------------------------------------
// Baud-rate candidates for the Phase 0 sweep.
// Most likely states first: factory default (9600), then our own target
// (TARGET_BAUD_RATE) in case a previous run partially succeeded.
// ---------------------------------------------------------------------------
const uint32_t UbxConfigurator::BAUD_CANDIDATES[UbxConfigurator::NUM_BAUD_CANDIDATES] = {
  9600UL, 115200UL, 38400UL, 57600UL, 19200UL, 230400UL
};

// Receive buffer large enough for any single UBX frame processed here.
// MON-VER with ten extension strings: ~6 + 40 + 300 = 346 bytes.  512 is ample.
static constexpr uint16_t RX_BUF_LEN = 512U;

// ===========================================================================
// Public entry point
// ===========================================================================

/**
 * @brief Runs the full Phase 0 / 1 / 2 configuration sequence.
 * @details Initialises the result struct to ERR_BAUD_NOT_FOUND so that an early
 *          return always yields a defined, meaningful status.  Phase 2 branches
 *          on m_generation: UBX_M9_PLUS takes the valset path; all other values
 *          take the legacy path.  m_generation is resolved from the MON-VER
 *          protocol version when UNKNOWN was passed; the caller's value is retained
 *          (and MON-VER is still polled for diagnostics) when a specific generation
 *          was supplied.
 */
UbxConfigResult UbxConfigurator::configure(HardwareSerial& serial,
                                            int8_t rxPin, int8_t txPin,
                                            GpsProvider generation)
{
  m_rxPin           = rxPin;
  m_txPin           = txPin;
  m_protocolVersion = 0U;
  m_generation      = generation;

  UbxConfigResult result{};
  result.status           = UbxConfigStatus::ERR_BAUD_NOT_FOUND;
  result.detectedProvider = GpsProvider::UNKNOWN;
  result.detectedBaud     = 0UL;
  result.protocolVersion  = 0U;
  result.validationPassed = false;

  // Phase 0: sweep all baud-rate candidates.
  //          Port is reopened at TARGET_BAUD_RATE on return.
  detectBaudRate(serial);

  // Phase 1: poll MON-VER.
  //   Always performed — proves the sweep landed and the module is responding
  //   at TARGET_BAUD_RATE.  Also extracts the protocol version, which is used
  //   to resolve m_generation when UNKNOWN was passed, and is reported in the
  //   result for diagnostics regardless.
  if (!identifyModule(serial)) {
    return result;  // status remains ERR_BAUD_NOT_FOUND
  }

  result.protocolVersion = m_protocolVersion;
  result.detectedBaud    = TARGET_BAUD_RATE;

  // Resolve generation from protocol version only when the caller did not
  // declare one at construction.
  if (m_generation == GpsProvider::UNKNOWN) {
    if (m_protocolVersion <= UBX_PROTO_VER_M6_MAX) {
      m_generation = GpsProvider::UBX_M6_MINUS;
    } else if (m_protocolVersion < UBX_PROTO_VER_VALSET_MIN) {
      m_generation = GpsProvider::UBX_M7_M8;
    } else {
      m_generation = GpsProvider::UBX_M9_PLUS;
    }
  }
  result.detectedProvider = m_generation;

  // Phase 2: run the appropriate configuration path.
  const UbxConfigStatus cfgStatus =
    (m_generation == GpsProvider::UBX_M9_PLUS)
      ? configureValset(serial)
      : configureLegacy(serial);

  if (cfgStatus != UbxConfigStatus::OK) {
    result.status = cfgStatus;
    return result;
  }

  result.validationPassed = true;
  result.status           = UbxConfigStatus::OK;
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
void UbxConfigurator::openSerial(HardwareSerial& serial, uint32_t baud)
{
  serial.end();
  serial.begin(baud, SERIAL_8N1, m_rxPin, m_txPin);
}

/**
 * @brief Discards all pending receive bytes for approximately 50 ms.
 * @details The 50 ms window is chosen to outlast any in-flight NMEA sentence
 *          (≤ ~40 ms at 9600 baud) so the buffer is clean before the next poll.
 */
void UbxConfigurator::flushRx(HardwareSerial& serial)
{
  const uint32_t deadline = millis() + 50U;
  while (millis() < deadline) {
    while (serial.available() > 0) {
      (void)serial.read();
    }
  }
}

/**
 * @brief Reads one complete, checksum-verified UBX frame from the serial port.
 * @details Runs an inline UBX frame state machine.  Frames whose declared payload
 *          length would exceed maxLen are discarded and the search continues.
 *          Frames with a bad Fletcher-8 checksum are also discarded.  The function
 *          returns as soon as one valid frame is found or the timeout expires.
 *          The local State enum mirrors UbxParser's UbxState; it is declared here
 *          to keep the configurator self-contained without a parser dependency.
 */
uint16_t UbxConfigurator::readUbxFrame(HardwareSerial& serial,
                                       uint8_t* buf, uint16_t maxLen,
                                       uint32_t timeoutMs)
{
  enum State : uint8_t {
    S_SYNC1, S_SYNC2, S_CLASS, S_ID,
    S_LEN_L, S_LEN_H, S_PAYLOAD, S_CK_A, S_CK_B
  };

  State    state      = S_SYNC1;
  uint16_t pos        = 0U;
  uint16_t payloadLen = 0U;
  uint16_t payloadIdx = 0U;
  uint8_t  ckA        = 0U;
  uint8_t  ckB        = 0U;

  const uint32_t deadline = millis() + timeoutMs;

  while (millis() < deadline) {
    if (serial.available() <= 0) { continue; }
    const uint8_t b = static_cast<uint8_t>(serial.read());

    switch (state) {
      case S_SYNC1:
        if (b == UBX_SYNC1) {
          pos = 0U; ckA = 0U; ckB = 0U;
          buf[pos++] = b;
          state = S_SYNC2;
        }
        break;

      case S_SYNC2:
        if (b == UBX_SYNC2) {
          buf[pos++] = b;
          state = S_CLASS;
        } else {
          // A second 0xB5 may be the start of a real frame; any other byte resets.
          state = (b == UBX_SYNC1) ? S_SYNC2 : S_SYNC1;
          pos = 0U;
        }
        break;

      case S_CLASS:
        buf[pos++] = b;
        // Initialise Fletcher-8 checksums at the start of the covered region.
        ckA = 0U;
        ckB = 0U;
        ckA += b;
        ckB += ckA;
        state = S_ID;
        break;

      case S_ID:
        buf[pos++] = b;
        ckA += b;
        ckB += ckA;
        state = S_LEN_L;
        break;

      case S_LEN_L:
        buf[pos++] = b;
        ckA += b;
        ckB += ckA;
        payloadLen = static_cast<uint16_t>(b);
        state = S_LEN_H;
        break;

      case S_LEN_H:
        buf[pos++] = b;
        ckA += b;
        ckB += ckA;
        payloadLen |= static_cast<uint16_t>(static_cast<uint32_t>(b) << 8U);
        if ((static_cast<uint16_t>(payloadLen + 8U)) > maxLen) {
          // Frame too large for buf — discard and resync.
          state = S_SYNC1; pos = 0U;
        } else if (payloadLen == 0U) {
          state = S_CK_A;
        } else {
          payloadIdx = 0U;
          state = S_PAYLOAD;
        }
        break;

      case S_PAYLOAD:
        buf[pos++] = b;
        ckA += b;
        ckB += ckA;
        if (++payloadIdx == payloadLen) { state = S_CK_A; }
        break;

      case S_CK_A:
        buf[pos++] = b;
        state = (b == ckA) ? S_CK_B : S_SYNC1;
        if (state == S_SYNC1) { pos = 0U; }
        break;

      case S_CK_B:
        buf[pos++] = b;
        if (b == ckB) { return pos; }
        // Checksum mismatch — discard and resync.
        state = S_SYNC1;
        pos   = 0U;
        break;

      default:
        state = S_SYNC1;
        pos   = 0U;
        break;
    }
  }
  return 0U;
}

/**
 * @brief Reads UBX frames, discarding any that do not match the expected class and ID.
 * @details Frames with non-matching class or ID consume some of the remaining timeout
 *          budget but are otherwise harmless.  Returns as soon as a matching frame is
 *          found or the deadline expires.
 */
uint16_t UbxConfigurator::readExpectedFrame(HardwareSerial& serial,
                                            uint8_t expectCls, uint8_t expectId,
                                            uint8_t* buf, uint16_t maxLen,
                                            uint32_t timeoutMs)
{
  const uint32_t deadline = millis() + timeoutMs;

  while (true) {
    const uint32_t now = millis();
    if (now >= deadline) { return 0U; }

    const uint16_t rxLen = readUbxFrame(serial, buf, maxLen, deadline - now);
    if (rxLen == 0U) { return 0U; }
    if ((buf[2] == expectCls) && (buf[3] == expectId)) { return rxLen; }
    // Valid frame but wrong type — keep waiting.
  }
}

/**
 * @brief Waits for a UBX-ACK-ACK or UBX-ACK-NAK matching the specified command.
 * @details All non-ACK frames and ACKs for other commands are silently discarded.
 *          A minimum frame length of 10 bytes is checked before accessing the
 *          payload bytes that carry the acknowledged class and ID.
 */
bool UbxConfigurator::waitForAck(HardwareSerial& serial,
                                 uint8_t expectCls, uint8_t expectId,
                                 uint32_t timeoutMs)
{
  uint8_t buf[16];
  const uint32_t deadline = millis() + timeoutMs;

  while (true) {
    const uint32_t now = millis();
    if (now >= deadline) { return false; }

    const uint16_t rxLen = readUbxFrame(serial, buf, sizeof(buf), deadline - now);
    if (rxLen < 10U) { continue; }
    if (buf[2] != UBX_CLASS_ACK) { continue; }
    if (buf[3] == UBX_ID_ACK_NAK) { return false; }
    if (buf[3] != UBX_ID_ACK_ACK) { continue; }
    if ((buf[6] == expectCls) && (buf[7] == expectId)) { return true; }
    // ACK for a different command — discard and keep waiting.
  }
}

// ===========================================================================
// Phase 0: blind baud-rate sweep
// ===========================================================================

/**
 * @brief Sends one open-channel burst: PUBX,41 + CFG-VALSET + CFG-PRT.
 * @details This is a file-scope helper called by detectBaudRate(); it is not a
 *          class member.  No acknowledgement is expected or awaited.  Each of the
 *          three commands targets a different module generation:
 *            PUBX,41   — accepted by all u-blox modules regardless of input protocol
 *                        state; re-enables UBX binary input for the commands that follow.
 *            CFG-VALSET — M9/M10 path: sets IO protocol masks and baud rate via RAM layer.
 *            CFG-PRT    — M6/M7/M8 path: sets IO protocol masks and baud rate.
 *          Only the RAM layer is written during the sweep; flash/BBR persistence is
 *          deferred to Phase 2 to avoid repeated wear from failed sweeps.
 */
static void sendOpenBurst(HardwareSerial& serial)
{
  uint8_t  txBuf[UbxMessageBuilder::MAX_TX_FRAME];
  uint8_t  kvBuf[UBX_CFGVALSET_MAX_KV_LEN];
  uint16_t txLen;
  uint16_t kvPos;

  // 1. PUBX,41
  txLen = UbxMessageBuilder::buildPubx41(
    UBX_UART1_PORT_ID,
    static_cast<uint16_t>(UBX_PROTO_MASK_UBX | UBX_PROTO_MASK_NMEA),
    UBX_PROTO_MASK_UBX,
    UbxConfigurator::TARGET_BAUD_RATE,
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));
  if (txLen > 0U) {
    serial.write(txBuf, txLen);
    serial.flush();
  }

  // 2. CFG-VALSET (RAM layer only — no flash writes during the sweep)
  kvPos = 0U;
  kvPos = UbxMessageBuilder::appendKeyU1(kvBuf, kvPos, UBXKEY_UART1INPROT_UBX,   1U);
  kvPos = UbxMessageBuilder::appendKeyU1(kvBuf, kvPos, UBXKEY_UART1INPROT_NMEA,  1U);
  kvPos = UbxMessageBuilder::appendKeyU1(kvBuf, kvPos, UBXKEY_UART1OUTPROT_UBX,  1U);
  kvPos = UbxMessageBuilder::appendKeyU1(kvBuf, kvPos, UBXKEY_UART1OUTPROT_NMEA, 0U);
  kvPos = UbxMessageBuilder::appendKeyU4(kvBuf, kvPos, UBXKEY_UART1_BAUDRATE,
                                          UbxConfigurator::TARGET_BAUD_RATE);

  txLen = UbxMessageBuilder::buildCfgValset(
    UBX_VALSET_LAYER_RAM, kvBuf, kvPos,
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));
  if (txLen > 0U) {
    serial.write(txBuf, txLen);
    serial.flush();
  }

  // 3. CFG-PRT (legacy M6/M7/M8 path)
  txLen = UbxMessageBuilder::buildCfgPrt(
    UBX_UART1_PORT_ID,
    UbxConfigurator::TARGET_BAUD_RATE,
    static_cast<uint16_t>(UBX_PROTO_MASK_UBX | UBX_PROTO_MASK_NMEA),
    UBX_PROTO_MASK_UBX,
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));
  if (txLen > 0U) {
    serial.write(txBuf, txLen);
    serial.flush();
  }
}

/**
 * @brief Iterates over BAUD_CANDIDATES, sending a double open-channel burst at each rate.
 * @details The burst is sent twice at each baud rate because the first attempt may
 *          arrive while the module UART receive FIFO is full following a previous
 *          command.  The port is reopened at TARGET_BAUD_RATE and flushed on return
 *          so Phase 1 can begin polling immediately.
 */
void UbxConfigurator::detectBaudRate(HardwareSerial& serial)
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
// Phase 1: contact verification and module identification
// ===========================================================================

/**
 * @brief Polls UBX-MON-VER to confirm contact and read the protocol version.
 * @details Two attempts are made: the module may still be draining queued output
 *          from the Phase 0 sweep, so the first poll may time out.  A fresh flush
 *          precedes each attempt.  On success m_protocolVersion is populated and,
 *          when m_generation is UNKNOWN, it is resolved from the version number.
 */
bool UbxConfigurator::identifyModule(HardwareSerial& serial)
{
  uint8_t  txBuf[UbxMessageBuilder::MAX_TX_FRAME];
  uint8_t  rxBuf[RX_BUF_LEN];

  const uint16_t pollLen = UbxMessageBuilder::buildPoll(
    UBX_CLASS_MON, UBX_ID_MON_VER,
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));

  // Two attempts: the module may still be draining queued output after the
  // sweep, so retry once after a fresh flush if the first poll times out.
  for (uint8_t attempt = 0U; attempt < 2U; ++attempt) {
    flushRx(serial);
    serial.write(txBuf, pollLen);
    serial.flush();

    const uint16_t rxLen = readExpectedFrame(
      serial, UBX_CLASS_MON, UBX_ID_MON_VER,
      rxBuf, static_cast<uint16_t>(sizeof(rxBuf)),
      MONVER_TIMEOUT_MS);

    if (rxLen > 0U) {
      m_protocolVersion = parseProtocolVersion(rxBuf, rxLen);
      return true;
    }
  }
  return false;
}

/**
 * @brief Parses the major protocol version from a raw MON-VER frame buffer.
 * @details MON-VER payload: char swVersion[30], char hwVersion[10], then
 *          N × char extension[30].  Extension strings beginning with "PROTVER"
 *          carry the version.  Only the leading decimal integer before any '.'
 *          is extracted; minor versions and sub-versions are not captured here.
 *          Some MON-VER payloads do not carry a "PROTVER" message this method
 *          returns 0 in that case and the module version is likely 6M or earlier.
 */
uint8_t UbxConfigurator::parseProtocolVersion(const uint8_t* buf, uint16_t bufLen)
{
  // MON-VER payload: char swVersion[30], char hwVersion[10],
  //                  then N × char extension[30].
  // Extension strings beginning with "PROTVER=" carry the version number.
  if (bufLen < static_cast<uint16_t>(6U + UBX_MONVER_MIN_PAYLOAD)) { return 0U; }

  const uint16_t payloadLen = static_cast<uint16_t>(buf[4])
                            | static_cast<uint16_t>(static_cast<uint32_t>(buf[5]) << 8U);
  if (payloadLen < UBX_MONVER_MIN_PAYLOAD) { return 0U; }

  const uint8_t* payload  = buf + 6U;
  const uint16_t extStart = static_cast<uint16_t>(UBX_MONVER_SWVER_LEN
                                                 + UBX_MONVER_HWVER_LEN);
  const uint16_t extCount = static_cast<uint16_t>(
    (payloadLen - extStart) / UBX_MONVER_EXT_LEN);

  for (uint16_t i = 0U; i < extCount; ++i) {
    const char* ext = reinterpret_cast<const char*>(
      payload + extStart + (i * static_cast<uint16_t>(UBX_MONVER_EXT_LEN)));

    if (strncmp(ext, "PROTVER", 7U) == 0) {
      const char* p = ext + 7U;
      // Skip non-digits (like '=' or spaces) to find the start of the version number.
      const char* limit = ext + static_cast<uint16_t>(UBX_MONVER_EXT_LEN);
      while ((p < limit) && ((*p < '0') || (*p > '9'))) {
        ++p;
      }

      uint8_t major = 0U;
      while ((p < limit) && (*p >= '0') && (*p <= '9')) {
        major = static_cast<uint8_t>((major * 10U) + static_cast<uint8_t>(*p - '0'));
        ++p;
      }
      return major;
    }
  }
  return 0U;
}

// ===========================================================================
// Phase 2: legacy configuration path (UBX_M6_MINUS and UBX_M7_M8)
// ===========================================================================

/**
 * @brief Configures M6 / M7 / M8 modules using legacy CFG-PRT / RATE / MSG / CFG.
 * @details Steps in order:
 *          1. CFG-PRT: confirm IO protocol (UBX+NMEA in, UBX out) at TARGET_BAUD_RATE.
 *             One automatic retry is attempted on first failure, since the module may
 *             still be processing residual output from the sweep.
 *          2. CFG-RATE: attempt 10 Hz (TARGET_MEAS_RATE_MS); fall back to 1 Hz
 *             (FALLBACK_RATE_MS) if the module rejects the faster rate.
 *          3. CFG-MSG: enable the appropriate message set for m_generation —
 *             NAV-POSLLH + NAV-SOL + NAV-VELNED for M6, NAV-PVT for M7/M8.
 *          4. CFG-CFG: save all subsections to all available non-volatile storage.
 *          5. validateLegacy(): read back CFG-PRT and CFG-RATE to confirm.
 */
UbxConfigStatus UbxConfigurator::configureLegacy(HardwareSerial& serial)
{
  uint8_t  txBuf[UbxMessageBuilder::MAX_TX_FRAME];
  uint16_t txLen;

  // CFG-PRT: confirm IO protocol.  Phase 0 already opened the channel but
  // this persists the setting (CFG-CFG save follows at the end).
  txLen = UbxMessageBuilder::buildCfgPrt(
    UBX_UART1_PORT_ID,
    TARGET_BAUD_RATE,
    static_cast<uint16_t>(UBX_PROTO_MASK_UBX | UBX_PROTO_MASK_NMEA),
    UBX_PROTO_MASK_UBX,
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));

  serial.write(txBuf, txLen);
  serial.flush();
  if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_PRT, ACK_TIMEOUT_MS)) {
    // One retry.
    serial.write(txBuf, txLen);
    serial.flush();
    if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_PRT, ACK_TIMEOUT_MS)) {
      return UbxConfigStatus::ERR_PROTO_FINAL_FAILED;
    }
  }
  delay(POST_CMD_DELAY_MS);

  // Navigation rate: attempt 10 Hz, fall back to 1 Hz.
  txLen = UbxMessageBuilder::buildCfgRate(
    TARGET_MEAS_RATE_MS, 1U, 0U,
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));
  serial.write(txBuf, txLen);
  serial.flush();
  if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_RATE, ACK_TIMEOUT_MS)) {
    txLen = UbxMessageBuilder::buildCfgRate(
      FALLBACK_RATE_MS, 1U, 0U,
      txBuf, static_cast<uint16_t>(sizeof(txBuf)));
    serial.write(txBuf, txLen);
    serial.flush();
    if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_RATE, ACK_TIMEOUT_MS)) {
      return UbxConfigStatus::ERR_RATE_FAILED;
    }
  }
  delay(POST_CMD_DELAY_MS);

  // Message enables — branch on declared generation, not protocol version.
  if (m_generation == GpsProvider::UBX_M6_MINUS) {
    // M6: enable the three-message epoch set.
    txLen = UbxMessageBuilder::buildCfgMsg(
      UBX_CLASS_NAV, UBX_ID_NAV_POSLLH, 1U,
      txBuf, static_cast<uint16_t>(sizeof(txBuf)));
    serial.write(txBuf, txLen);
    serial.flush();
    if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_MSG, ACK_TIMEOUT_MS)) {
      return UbxConfigStatus::ERR_MSG_FAILED;
    }
    delay(POST_CMD_DELAY_MS);

    txLen = UbxMessageBuilder::buildCfgMsg(
      UBX_CLASS_NAV, UBX_ID_NAV_SOL, 1U,
      txBuf, static_cast<uint16_t>(sizeof(txBuf)));
    serial.write(txBuf, txLen);
    serial.flush();
    if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_MSG, ACK_TIMEOUT_MS)) {
      return UbxConfigStatus::ERR_MSG_FAILED;
    }
    delay(POST_CMD_DELAY_MS);

    txLen = UbxMessageBuilder::buildCfgMsg(
      UBX_CLASS_NAV, UBX_ID_NAV_VELNED, 1U,
      txBuf, static_cast<uint16_t>(sizeof(txBuf)));
    serial.write(txBuf, txLen);
    serial.flush();
    if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_MSG, ACK_TIMEOUT_MS)) {
      return UbxConfigStatus::ERR_MSG_FAILED;
    }
    delay(POST_CMD_DELAY_MS);

    // Enable NAV-TIMEUTC for UTC date/time (decoupled from epoch gate).
    txLen = UbxMessageBuilder::buildCfgMsg(
      UBX_CLASS_NAV, UBX_ID_NAV_TIMEUTC, 1U,
      txBuf, static_cast<uint16_t>(sizeof(txBuf)));
    serial.write(txBuf, txLen);
    serial.flush();
    if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_MSG, ACK_TIMEOUT_MS)) {
      return UbxConfigStatus::ERR_MSG_FAILED;
    }
    delay(POST_CMD_DELAY_MS);

  } else {
    // UBX_M7_M8: enable NAV-PVT.
    txLen = UbxMessageBuilder::buildCfgMsg(
      UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1U,
      txBuf, static_cast<uint16_t>(sizeof(txBuf)));
    serial.write(txBuf, txLen);
    serial.flush();
    if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_MSG, ACK_TIMEOUT_MS)) {
      return UbxConfigStatus::ERR_MSG_FAILED;
    }
    delay(POST_CMD_DELAY_MS);
  }

  // Save to all available non-volatile storage.
  txLen = UbxMessageBuilder::buildCfgCfgSave(
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));
  serial.write(txBuf, txLen);
  serial.flush();
  if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_CFG, SAVE_TIMEOUT_MS)) {
    return UbxConfigStatus::ERR_SAVE_FAILED;
  }
  delay(POST_CMD_DELAY_MS);

  return validateLegacy(serial);
}

// ===========================================================================
// Phase 2: new-style configuration path (UBX_M9_PLUS)
// ===========================================================================

/**
 * @brief Configures M9 / M10 modules with a single CFG-VALSET covering all targets.
 * @details All seven key-value pairs are written in one transaction to all storage
 *          layers (RAM | BBR | Flash).  No separate baud-rate key is included here
 *          because Phase 0 already set TARGET_BAUD_RATE in RAM; the persistent layers are
 *          handled by the LAYER_ALL write.  If the build fails (kvBuf overflow or
 *          frame assembly error), ERR_PROTO_FINAL_FAILED is returned before any
 *          bytes are sent to the module.
 */
UbxConfigStatus UbxConfigurator::configureValset(HardwareSerial& serial)
{
  uint8_t  txBuf[UbxMessageBuilder::MAX_TX_FRAME];
  uint8_t  kvBuf[UBX_CFGVALSET_MAX_KV_LEN];
  uint16_t txLen;
  uint16_t kvPos = 0U;

  // Single CFG-VALSET written to all layers: IO protocol, navigation rate,
  // and NAV-PVT output enable.  No baud-rate key — Phase 0 already set
  // TARGET_BAUD_RATE baud in RAM; the persistent layers are handled here.
  kvPos = UbxMessageBuilder::appendKeyU1(kvBuf, kvPos, UBXKEY_UART1INPROT_UBX,      1U);
  kvPos = UbxMessageBuilder::appendKeyU1(kvBuf, kvPos, UBXKEY_UART1INPROT_NMEA,     1U);
  kvPos = UbxMessageBuilder::appendKeyU1(kvBuf, kvPos, UBXKEY_UART1OUTPROT_UBX,     1U);
  kvPos = UbxMessageBuilder::appendKeyU1(kvBuf, kvPos, UBXKEY_UART1OUTPROT_NMEA,    0U);
  kvPos = UbxMessageBuilder::appendKeyU2(kvBuf, kvPos, UBXKEY_RATE_MEAS,            TARGET_MEAS_RATE_MS);
  kvPos = UbxMessageBuilder::appendKeyU2(kvBuf, kvPos, UBXKEY_RATE_NAV,             1U);
  kvPos = UbxMessageBuilder::appendKeyU1(kvBuf, kvPos, UBXKEY_MSGOUT_NAV_PVT_UART1, 1U);

  txLen = UbxMessageBuilder::buildCfgValset(
    UBX_VALSET_LAYER_ALL, kvBuf, kvPos,
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));

  if (txLen == 0U) { return UbxConfigStatus::ERR_PROTO_FINAL_FAILED; }

  serial.write(txBuf, txLen);
  serial.flush();
  if (!waitForAck(serial, UBX_CLASS_CFG, UBX_ID_CFG_VALSET, ACK_TIMEOUT_MS)) {
    return UbxConfigStatus::ERR_PROTO_FINAL_FAILED;
  }
  delay(POST_CMD_DELAY_MS);

  return validateValset(serial);
}

// ===========================================================================
// Post-configuration validation
// ===========================================================================

/**
 * @brief Reads back CFG-PRT and CFG-RATE to verify the legacy configuration targets.
 * @details CFG-PRT check: verifies that outProtoMask == UBX_PROTO_MASK_UBX (UBX only)
 *          and that the baud rate field equals TARGET_BAUD_RATE.
 *          CFG-RATE check: accepts either TARGET_MEAS_RATE_MS or FALLBACK_RATE_MS,
 *          since configureLegacy() may have accepted the 1 Hz fallback.
 *          Both fields are read using memcpy to avoid aliasing undefined behaviour.
 */
UbxConfigStatus UbxConfigurator::validateLegacy(HardwareSerial& serial)
{
  uint8_t  txBuf[UbxMessageBuilder::MAX_TX_FRAME];
  uint8_t  rxBuf[64];
  uint16_t txLen;

  // -------------------------------------------------------------------------
  // CFG-PRT readback — verify output protocol mask and baud rate.
  // -------------------------------------------------------------------------
  txLen = UbxMessageBuilder::buildPoll(
    UBX_CLASS_CFG, UBX_ID_CFG_PRT,
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));

  flushRx(serial);
  serial.write(txBuf, txLen);
  serial.flush();

  uint16_t rxLen = readExpectedFrame(
    serial, UBX_CLASS_CFG, UBX_ID_CFG_PRT,
    rxBuf, static_cast<uint16_t>(sizeof(rxBuf)),
    ACK_TIMEOUT_MS);

  if (rxLen < static_cast<uint16_t>(6U + UBX_CFGPRT_PAYLOAD_LEN)) {
    return UbxConfigStatus::ERR_VALIDATION_FAILED;
  }

  // payload[14..15] = outProtoMask (U2 LE)
  const uint16_t outProto =
      static_cast<uint16_t>(rxBuf[6U + 14U])
    | static_cast<uint16_t>(static_cast<uint32_t>(rxBuf[6U + 15U]) << 8U);

  // payload[8..11] = baud rate (U4 LE)
  uint32_t baud = 0U;
  (void)memcpy(&baud, &rxBuf[6U + 8U], 4U);

  if ((outProto != UBX_PROTO_MASK_UBX) || (baud != TARGET_BAUD_RATE)) {
    return UbxConfigStatus::ERR_VALIDATION_FAILED;
  }

  // -------------------------------------------------------------------------
  // CFG-RATE readback — verify navigation measurement rate.
  // Accept TARGET_MEAS_RATE_MS or FALLBACK_RATE_MS; both are valid outcomes
  // of configureLegacy() since it falls back to 1 Hz if 10 Hz is rejected.
  // -------------------------------------------------------------------------
  txLen = UbxMessageBuilder::buildPoll(
    UBX_CLASS_CFG, UBX_ID_CFG_RATE,
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));

  flushRx(serial);
  serial.write(txBuf, txLen);
  serial.flush();

  rxLen = readExpectedFrame(
    serial, UBX_CLASS_CFG, UBX_ID_CFG_RATE,
    rxBuf, static_cast<uint16_t>(sizeof(rxBuf)),
    ACK_TIMEOUT_MS);

  if (rxLen < static_cast<uint16_t>(6U + UBX_CFGRATE_PAYLOAD_LEN)) {
    return UbxConfigStatus::ERR_VALIDATION_FAILED;
  }

  // payload[0..1] = measRate in milliseconds (U2 LE)
  uint16_t measRate = 0U;
  (void)memcpy(&measRate, &rxBuf[6U], sizeof(measRate));

  const bool rateOk = (measRate == TARGET_MEAS_RATE_MS) ||
                      (measRate == FALLBACK_RATE_MS);
  if (!rateOk) {
    return UbxConfigStatus::ERR_VALIDATION_FAILED;
  }

  return UbxConfigStatus::OK;
}

/**
 * @brief Reads back three configuration keys via CFG-VALGET to verify the valset targets.
 * @details Queries UBXKEY_UART1_BAUDRATE (U4), UBXKEY_UART1OUTPROT_UBX (L, 1 byte),
 *          and UBXKEY_RATE_MEAS (U2) from the RAM layer.  Unknown keys in the response
 *          are skipped using the size code encoded in bits [31:28] of the key ID.
 *          FALLBACK_RATE_MS is accepted alongside TARGET_MEAS_RATE_MS for the rate key.
 */
UbxConfigStatus UbxConfigurator::validateValset(HardwareSerial& serial)
{
  uint8_t  txBuf[UbxMessageBuilder::MAX_TX_FRAME];
  uint8_t  rxBuf[64];

  // Read back the three settings most likely to be wrong after a failed
  // partial configuration: baud rate, output protocol, and navigation rate.
  static const uint32_t keys[3] = {
    UBXKEY_UART1_BAUDRATE,    // U4 — target 115 200
    UBXKEY_UART1OUTPROT_UBX,  // L (1 byte) — target 1
    UBXKEY_RATE_MEAS          // U2 — target TARGET_MEAS_RATE_MS
  };

  const uint16_t txLen = UbxMessageBuilder::buildCfgValget(
    UBX_VALGET_LAYER_RAM, keys, 3U,
    txBuf, static_cast<uint16_t>(sizeof(txBuf)));

  flushRx(serial);
  serial.write(txBuf, txLen);
  serial.flush();

  const uint16_t rxLen = readExpectedFrame(
    serial, UBX_CLASS_CFG, UBX_ID_CFG_VALGET,
    rxBuf, static_cast<uint16_t>(sizeof(rxBuf)),
    ACK_TIMEOUT_MS);

  // Minimum: 6-byte frame header + 4-byte CFG-VALGET header + one U4 key + U4 value
  if (rxLen < static_cast<uint16_t>(6U + UBX_CFGVAL_HDR_LEN + 8U)) {
    return UbxConfigStatus::ERR_VALIDATION_FAILED;
  }

  const uint8_t* payload = rxBuf + 6U;
  const uint16_t payLen  =
      static_cast<uint16_t>(rxBuf[4])
    | static_cast<uint16_t>(static_cast<uint32_t>(rxBuf[5]) << 8U);

  uint16_t idx       = UBX_CFGVAL_HDR_LEN;
  bool     baudOk    = false;
  bool     outprotOk = false;
  bool     rateOk    = false;

  while (static_cast<uint16_t>(idx + 5U) <= payLen) {
    uint32_t key = 0U;
    (void)memcpy(&key, payload + idx, 4U);
    idx = static_cast<uint16_t>(idx + 4U);

    if (key == UBXKEY_UART1_BAUDRATE) {
      // U4 value — 4 bytes
      uint32_t val = 0U;
      (void)memcpy(&val, payload + idx, 4U);
      baudOk = (val == TARGET_BAUD_RATE);
      idx = static_cast<uint16_t>(idx + 4U);

    } else if (key == UBXKEY_UART1OUTPROT_UBX) {
      // L value — 1 byte; non-zero means enabled
      outprotOk = (payload[idx] != 0U);
      idx = static_cast<uint16_t>(idx + 1U);

    } else if (key == UBXKEY_RATE_MEAS) {
      // U2 value — 2 bytes
      uint16_t val = 0U;
      (void)memcpy(&val, payload + idx, 2U);
      // Accept TARGET or FALLBACK; configureValset always targets TARGET but
      // a module that silently clamped to 1 Hz is still usable.
      rateOk = (val == TARGET_MEAS_RATE_MS) || (val == FALLBACK_RATE_MS);
      idx = static_cast<uint16_t>(idx + 2U);

    } else {
      // Unknown key — advance past its value using the size code in bits [31:28].
      // Size codes: 0x1 = 1 byte (L/X1/U1), 0x2 = 2 bytes (U2/X2),
      //             0x3 = 4 bytes (U4/X4/R4), 0x4 = 8 bytes (U8/X8/R8).
      const uint8_t sizeCode = static_cast<uint8_t>((key >> 28U) & 0x0FU);
      uint8_t valueSize = 1U;
      if      (sizeCode == 0x2U) { valueSize = 2U; }
      else if (sizeCode == 0x3U) { valueSize = 4U; }
      else if (sizeCode == 0x4U) { valueSize = 8U; }
      else                       { /* L, X1, U1, or unrecognised: 1 byte */ }
      idx = static_cast<uint16_t>(idx + valueSize);
    }
  }

  return (baudOk && outprotOk && rateOk)
    ? UbxConfigStatus::OK
    : UbxConfigStatus::ERR_VALIDATION_FAILED;
}
