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

// ---------------------------------------------------------------------------
// CasicMessageBuilder
//
// Header-only utility namespace for constructing and validating CASIC binary
// frames.
//
// CASIC frame structure:
//   [0xBA][0xCE][len_lo][len_hi][class][id][payload...][ck0][ck1][ck2][ck3]
//
// len is the payload length in bytes (must be a multiple of 4).
//
// 32-bit checksum:
//   ckSum  = (id << 24) + (class << 16) + len
//   for each 4-byte little-endian word in the payload:
//       ckSum += word
// The checksum is transmitted little-endian.
//
// Frame overhead (excluding payload):
//   2 (header) + 2 (len) + 1 (class) + 1 (id) + 4 (checksum) = 10 bytes
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <string.h>  // memcpy

#include "CasicConstants.hpp"

// Maximum frame buffer size for any CFG message built by this namespace.
// Largest payload sent is CFG-PRT at CASIC_CFG_PRT_PAYLOAD_LEN (8 bytes)
// → 18 bytes total; 32 bytes gives comfortable margin.
static constexpr uint16_t CASIC_MAX_FRAME_BUF = 32U;

// CASIC frame overhead in bytes (excluding payload).
static constexpr uint8_t CASIC_FRAME_OVERHEAD = 10U;

namespace CasicMessageBuilder {

    /**
     * @brief Computes the 32-bit CASIC checksum for a frame.
     * @details The checksum seed packs the three non-overlapping header fields
     *          into a single uint32: id in bits 31–24, class in bits 23–16, and
     *          payloadLen in bits 15–0.  Each complete 4-byte little-endian word
     *          of the payload is then added to the accumulator.
     * @param cls     Message class byte.
     * @param id      Message ID byte.
     * @param len     Payload length in bytes (must be a multiple of 4).
     * @param payload Pointer to the raw payload bytes.
     * @return The 32-bit checksum value.
     */
    inline uint32_t computeChecksum(uint8_t cls, uint8_t id,
                                    uint16_t len, const uint8_t* payload)
    {
        // Non-overlapping fields packed into a uint32;
        uint32_t ck = (static_cast<uint32_t>(id)  << 24U)
                    | (static_cast<uint32_t>(cls)  << 16U)
                    |  static_cast<uint32_t>(len);

        const uint16_t nWords = len / 4U;
        for (uint16_t i = 0U; i < nWords; ++i) {
            uint32_t word = 0U;
            memcpy(&word, payload + (i * 4U), 4U);
            ck += word;
        }
        return ck;
    }

    /**
     * @brief Serialises a complete CASIC binary frame into the caller's output buffer.
     * @details Writes the two sync bytes, little-endian payload length, class, ID,
     *          payload bytes (if any), and the 32-bit checksum in little-endian order.
     *          payload may be nullptr when payloadLen == 0.
     * @param cls        Message class byte.
     * @param id         Message ID byte.
     * @param payload    Pointer to the payload bytes, or nullptr for a zero-payload frame.
     * @param payloadLen Payload length in bytes (must be a multiple of 4).
     * @param out        Caller-supplied output buffer; must be at least
     *                   (payloadLen + CASIC_FRAME_OVERHEAD) bytes.
     * @param outBufLen  Size of the output buffer in bytes.
     * @return Total bytes written, or 0 if the output buffer is too small.
     */
    inline uint16_t buildFrame(uint8_t cls, uint8_t id,
                               const uint8_t* payload, uint16_t payloadLen,
                               uint8_t* out, uint16_t outBufLen)
    {
        const uint16_t frameLen = CASIC_FRAME_OVERHEAD + payloadLen;
        if (frameLen > outBufLen) { return 0U; }

        uint16_t pos = 0U;

        out[pos++] = CASIC_SYNC1;
        out[pos++] = CASIC_SYNC2;

        // Payload length (little-endian U2)
        out[pos++] = static_cast<uint8_t>(payloadLen & 0xFFU);
        out[pos++] = static_cast<uint8_t>(payloadLen >> 8U);

        out[pos++] = cls;
        out[pos++] = id;

        if ((payloadLen > 0U) && (payload != nullptr)) {
            memcpy(out + pos, payload, payloadLen);
            pos += payloadLen;
        }

        // Checksum (little-endian U4)
        const uint32_t ck = computeChecksum(cls, id, payloadLen,
                                            ((payloadLen > 0U) ? payload : nullptr));
        out[pos++] = static_cast<uint8_t>( ck        & 0xFFU);
        out[pos++] = static_cast<uint8_t>((ck >>  8U) & 0xFFU);
        out[pos++] = static_cast<uint8_t>((ck >> 16U) & 0xFFU);
        out[pos++] = static_cast<uint8_t>((ck >> 24U) & 0xFFU);

        return pos;
    }

    /**
     * @brief Builds a zero-payload CASIC query frame.
     * @details Convenience wrapper around buildFrame() for poll / query messages
     *          such as a CFG-PRT query.  A zero-length payload is divisible by 4
     *          and therefore always passes the checksum alignment requirement.
     * @param cls       Message class byte.
     * @param id        Message ID byte.
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Total bytes written, or 0 if the output buffer is too small.
     */
    inline uint16_t buildQuery(uint8_t cls, uint8_t id,
                               uint8_t* out, uint16_t outBufLen)
    {
        return buildFrame(cls, id, nullptr, 0U, out, outBufLen);
    }

    /**
     * @brief Validates a complete CASIC frame: checks sync bytes, length field,
     *        and 32-bit checksum.
     * @param buf      Pointer to the frame buffer.
     * @param frameLen Total length of the frame in bytes.
     * @return true if the sync bytes are correct, the payload length field is
     *         consistent with frameLen, and the checksum matches; false otherwise.
     */
    inline bool validateFrame(const uint8_t* buf, uint16_t frameLen)
    {
        if (frameLen < CASIC_FRAME_OVERHEAD) { return false; }
        if ((buf[0] != CASIC_SYNC1) || (buf[1] != CASIC_SYNC2)) { return false; }

        const uint16_t payLen = static_cast<uint16_t>(buf[2])
                              | (static_cast<uint16_t>(buf[3]) << 8U);

        if (static_cast<uint16_t>(payLen + CASIC_FRAME_OVERHEAD) != frameLen) {
            return false;
        }

        const uint8_t  cls     = buf[4];
        const uint8_t  id      = buf[5];
        const uint8_t* payload = buf + 6U;

        const uint32_t calcCk = computeChecksum(cls, id, payLen, payload);

        uint32_t rxCk = 0U;
        memcpy(&rxCk, buf + 6U + payLen, 4U);  // little-endian U4

        return (calcCk == rxCk);
    }

    /**
     * @brief Extracts the acknowledged class and ID from a validated ACK or NACK frame.
     * @details buf must already have passed validateFrame().  Both ackedCls and
     *          ackedId are populated regardless of whether the frame is ACK or NACK.
     * @param buf      Pointer to the validated CASIC ACK / NACK frame.
     * @param ackedCls Output: class byte of the command being acknowledged.
     * @param ackedId  Output: message ID of the command being acknowledged.
     * @return true for ACK-ACK (command accepted), false for ACK-NACK (command rejected).
     */
    inline bool parseAck(const uint8_t* buf, uint8_t& ackedCls, uint8_t& ackedId)
    {
        // Payload starts at buf[6]: U1 clsID, U1 msgID, U2 reserved
        ackedCls = buf[6];
        ackedId  = buf[7];
        return (buf[5] == CASIC_ID_ACK_ACK);
    }

    /**
     * @brief Builds a CFG-PRT set frame to configure the active UART port.
     * @details Payload layout (CASIC_CFG_PRT_PAYLOAD_LEN = 8 bytes):
     *            offset 0: U1 portId
     *            offset 1: U1 protoMask
     *            offset 2: U2 uartMode (little-endian)
     *            offset 4: U4 baudRate (little-endian)
     * @param portId    Port to configure (CASIC_PORT_CURRENT = 0xFF for the active UART).
     * @param protoMask Protocol enable bitmask (see CASIC_PROTO_* constants).
     * @param uartMode  UART framing mode (CASIC_UART_MODE_8N1 = 0x08C0).
     * @param baudRate  Desired baud rate in bits/s.
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Total bytes written, or 0 if the output buffer is too small.
     */
    inline uint16_t buildCfgPrt(uint8_t portId, uint8_t protoMask,
                                uint16_t uartMode, uint32_t baudRate,
                                uint8_t* out, uint16_t outBufLen)
    {
        uint8_t payload[CASIC_CFG_PRT_PAYLOAD_LEN];
        payload[0] = portId;
        payload[1] = protoMask;
        payload[2] = static_cast<uint8_t>( uartMode        & 0xFFU);
        payload[3] = static_cast<uint8_t>((uartMode >> 8U) & 0xFFU);
        payload[4] = static_cast<uint8_t>( baudRate         & 0xFFU);
        payload[5] = static_cast<uint8_t>((baudRate >>  8U) & 0xFFU);
        payload[6] = static_cast<uint8_t>((baudRate >> 16U) & 0xFFU);
        payload[7] = static_cast<uint8_t>((baudRate >> 24U) & 0xFFU);

        return buildFrame(CASIC_CLASS_CFG, CASIC_ID_CFG_PRT,
                          payload, CASIC_CFG_PRT_PAYLOAD_LEN, out, outBufLen);
    }

    /**
     * @brief Builds a CFG-RATE set frame to configure the navigation update interval.
     * @details Payload layout (CASIC_CFG_RATE_PAYLOAD_LEN = 4 bytes):
     *            offset 0: U2 intervalMs (little-endian)
     *            offset 2: U2 reserved (zero)
     * @param intervalMs  Navigation update interval in milliseconds (e.g. 100 for 10 Hz).
     * @param out         Caller-supplied output buffer.
     * @param outBufLen   Size of the output buffer in bytes.
     * @return Total bytes written, or 0 if the output buffer is too small.
     */
    inline uint16_t buildCfgRate(uint16_t intervalMs,
                                 uint8_t* out, uint16_t outBufLen)
    {
        uint8_t payload[CASIC_CFG_RATE_PAYLOAD_LEN];
        payload[0] = static_cast<uint8_t>( intervalMs        & 0xFFU);
        payload[1] = static_cast<uint8_t>((intervalMs >> 8U) & 0xFFU);
        payload[2] = 0x00U;
        payload[3] = 0x00U;

        return buildFrame(CASIC_CLASS_CFG, CASIC_ID_CFG_RATE,
                          payload, CASIC_CFG_RATE_PAYLOAD_LEN, out, outBufLen);
    }

    /**
     * @brief Builds a CFG-MSG set frame to configure the output rate of a message.
     * @details Payload layout (CASIC_CFG_MSG_PAYLOAD_LEN = 4 bytes):
     *            offset 0: U1 msgCls
     *            offset 1: U1 msgId
     *            offset 2: U2 rate (little-endian)
     * @param msgCls    Class of the message to configure (e.g. CASIC_CLASS_NAV).
     * @param msgId     ID of the message to configure (e.g. CASIC_ID_NAV_PV).
     * @param rate      Output rate: 0 = disabled, 1 = every epoch, N = every Nth epoch.
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Total bytes written, or 0 if the output buffer is too small.
     */
    inline uint16_t buildCfgMsg(uint8_t msgCls, uint8_t msgId,
                                uint16_t rate,
                                uint8_t* out, uint16_t outBufLen)
    {
        uint8_t payload[CASIC_CFG_MSG_PAYLOAD_LEN];
        payload[0] = msgCls;
        payload[1] = msgId;
        payload[2] = static_cast<uint8_t>( rate        & 0xFFU);
        payload[3] = static_cast<uint8_t>((rate >> 8U) & 0xFFU);

        return buildFrame(CASIC_CLASS_CFG, CASIC_ID_CFG_MSG,
                          payload, CASIC_CFG_MSG_PAYLOAD_LEN, out, outBufLen);
    }

    /**
     * @brief Builds a CFG-CFG command frame to save or load configuration groups.
     * @details Payload layout (CASIC_CFG_CFG_PAYLOAD_LEN = 4 bytes):
     *            offset 0: U2 mask (little-endian) — selects configuration groups
     *            offset 2: U1 mode — 0 = clear, 1 = save, 2 = load
     *            offset 3: U1 reserved (zero)
     *          Use CASIC_CFG_SAVE_MASK and CASIC_CFG_MODE_SAVE to persist all groups.
     * @param mask      Bitfield selecting which configuration groups to operate on.
     * @param mode      Operation mode: 0 = clear, 1 = save, 2 = load.
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Total bytes written, or 0 if the output buffer is too small.
     */
    inline uint16_t buildCfgCfg(uint16_t mask, uint8_t mode,
                                 uint8_t* out, uint16_t outBufLen)
    {
        uint8_t payload[CASIC_CFG_CFG_PAYLOAD_LEN];
        payload[0] = static_cast<uint8_t>( mask        & 0xFFU);
        payload[1] = static_cast<uint8_t>((mask >> 8U) & 0xFFU);
        payload[2] = mode;
        payload[3] = 0x00U;

        return buildFrame(CASIC_CLASS_CFG, CASIC_ID_CFG_CFG,
                          payload, CASIC_CFG_CFG_PAYLOAD_LEN, out, outBufLen);
    }

    /**
     * @brief Extracts the protocol mask and baud rate from a validated CFG-PRT response.
     * @details buf must already have passed validateFrame() and must be a CFG-PRT
     *          response frame starting at the CASIC sync bytes.
     * @param buf           Pointer to the validated CFG-PRT response frame.
     * @param outProtoMask  Output: protocol enable bitmask read from the response.
     * @param outBaudRate   Output: baud rate read from the response.
     */
    inline void parseCfgPrtResponse(const uint8_t* buf,
                                    uint8_t&  outProtoMask,
                                    uint32_t& outBaudRate)
    {
        const uint8_t* payload = buf + 6U;  // skip header(2), len(2), cls(1), id(1)
        outProtoMask = payload[1];
        memcpy(&outBaudRate, payload + 4U, 4U);  // little-endian U4
    }

    /**
     * @brief Extracts the navigation update interval from a validated CFG-RATE response.
     * @details buf must already have passed validateFrame() and must be a CFG-RATE
     *          response frame starting at the CASIC sync bytes.
     * @param buf Pointer to the validated CFG-RATE response frame.
     * @return Navigation update interval in milliseconds.
     */
    inline uint16_t parseCfgRateResponse(const uint8_t* buf)
    {
        const uint8_t* payload = buf + 6U;
        uint16_t interval = 0U;
        memcpy(&interval, payload, 2U);  // little-endian U2
        return interval;
    }

} // namespace CasicMessageBuilder
