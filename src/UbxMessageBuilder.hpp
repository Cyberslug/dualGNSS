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
// UbxMessageBuilder
//
// Header-only utility namespace that constructs UBX binary frames and the
// PUBX,41 NMEA sentence used during the phase-A protocol-open step.
//
// All build functions write into a caller-supplied byte buffer and return
// the total number of bytes written (frame length including sync bytes and
// checksum).  A return value of zero indicates the output buffer was too
// small.
//
// UBX frame layout (total = 6 + payloadLen bytes):
//   Offset  Size  Field
//      0      1   SYNC1  = 0xB5
//      1      1   SYNC2  = 0x62
//      2      1   Class
//      3      1   ID
//      4      2   Length (payload only, little-endian)
//      6    len   Payload
//   6+len    1   CK_A   (Fletcher-8 over bytes 2…5+len)
//   7+len    1   CK_B
//
// CFG-VALSET key-value encoding helpers (appendKeyU1/U2/U4) are provided so
// that the caller can build the cfgData buffer before calling buildCfgValset().
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <string.h>  // memset, memcpy
#include <stdio.h>   // snprintf

#include "UbxConstants.hpp"

namespace UbxMessageBuilder {

    // -----------------------------------------------------------------------
    // detail — private little-endian write helpers.
    // Not part of the public API; use the build* functions instead.
    // -----------------------------------------------------------------------
    namespace detail {

        /**
         * @brief Writes a uint16_t value in little-endian byte order to dst[0..1].
         * @details Explicit byte extraction avoids strict-aliasing violations that
         *          would arise from casting dst to uint16_t* and dereferencing.
         * @param dst Pointer to a 2-byte destination region.
         * @param val The value to write.
         */
        inline void writeU2LE(uint8_t* dst, uint16_t val)
        {
            dst[0] = static_cast<uint8_t>(val & 0xFFU);
            dst[1] = static_cast<uint8_t>((val >> 8U) & 0xFFU);
        }

        /**
         * @brief Writes a uint32_t value in little-endian byte order to dst[0..3].
         * @details Explicit byte extraction avoids strict-aliasing violations.
         * @param dst Pointer to a 4-byte destination region.
         * @param val The value to write.
         */
        inline void writeU4LE(uint8_t* dst, uint32_t val)
        {
            dst[0] = static_cast<uint8_t>(val & 0xFFU);
            dst[1] = static_cast<uint8_t>((val >>  8U) & 0xFFU);
            dst[2] = static_cast<uint8_t>((val >> 16U) & 0xFFU);
            dst[3] = static_cast<uint8_t>((val >> 24U) & 0xFFU);
        }

    } // namespace detail

    // Maximum size for output (TX) frames.
    // CFG-VALSET worst case: 4-byte header + 7 key-value pairs (up to 8 bytes
    // each) ≈ 60 bytes payload → 6 + 60 + 2 = 68 bytes.  128 bytes is ample.
    inline constexpr uint16_t MAX_TX_FRAME = 128U;

    /**
     * @brief Computes the UBX Fletcher-8 checksum over the frame body region.
     * @details Iterates over buf[2 .. frameLen-3], covering the class, ID, length,
     *          and payload bytes — the region the UBX protocol defines as subject
     *          to the checksum.  The two sync bytes and the two trailing checksum
     *          bytes are excluded.
     * @param buf      Pointer to a complete UBX frame of exactly frameLen bytes.
     * @param frameLen Total length of the frame in bytes.
     * @param ckA      Output: Fletcher-8 accumulator A (byte at frameLen-2).
     * @param ckB      Output: Fletcher-8 accumulator B (byte at frameLen-1).
     */
    inline void computeChecksum(const uint8_t* buf, uint16_t frameLen,
                                uint8_t& ckA, uint8_t& ckB)
    {
        ckA = 0U;
        ckB = 0U;
        for (uint16_t i = 2U; i < (frameLen - 2U); ++i) {
            ckA += buf[i];
            ckB += ckA;
        }
    }

    /**
     * @brief Validates a UBX frame by checking its sync bytes and Fletcher-8 checksum.
     * @param buf Pointer to the frame buffer containing frameLen bytes.
     * @param len Total length of the frame in bytes (minimum 8 for a valid frame).
     * @return true if the sync bytes are correct and the checksum matches, false otherwise.
     */
    inline bool validateFrame(const uint8_t* buf, uint16_t len)
    {
        if (len < 8U) { return false; }
        if ((buf[0] != UBX_SYNC1) || (buf[1] != UBX_SYNC2)) { return false; }

        uint8_t ckA = 0U;
        uint8_t ckB = 0U;
        computeChecksum(buf, len, ckA, ckB);
        return (buf[len - 2U] == ckA) && (buf[len - 1U] == ckB);
    }

    /**
     * @brief Assembles a complete UBX binary frame into the caller's output buffer.
     * @details Writes the two sync bytes, class, ID, little-endian payload length,
     *          payload bytes (if any), and the Fletcher-8 checksum pair.
     *          payload may be nullptr when payloadLen == 0.
     * @param cls        Message class byte.
     * @param id         Message ID byte.
     * @param payload    Pointer to the payload data, or nullptr for a zero-payload frame.
     * @param payloadLen Length of the payload in bytes.
     * @param out        Caller-supplied output buffer.
     * @param outBufLen  Size of the output buffer in bytes.
     * @return Total frame length (6 + payloadLen + 2) in bytes, or 0 if out is too small.
     */
    inline uint16_t buildFrame(uint8_t cls, uint8_t id,
                               const uint8_t* payload, uint16_t payloadLen,
                               uint8_t* out, uint16_t outBufLen)
    {
        const uint16_t frameLen = 8U + payloadLen;
        if (frameLen > outBufLen) { return 0U; }

        out[0] = UBX_SYNC1;
        out[1] = UBX_SYNC2;
        out[2] = cls;
        out[3] = id;
        out[4] = static_cast<uint8_t>(payloadLen & 0xFFU);
        out[5] = static_cast<uint8_t>((payloadLen >> 8U) & 0xFFU);

        if ((payloadLen > 0U) && (payload != nullptr)) {
            memcpy(&out[6], payload, payloadLen);
        }

        computeChecksum(out, frameLen, out[frameLen - 2U], out[frameLen - 1U]);
        return frameLen;
    }

    /**
     * @brief Builds a zero-payload UBX poll / query frame.
     * @details Convenience wrapper around buildFrame() for messages that carry no
     *          payload, such as a UBX-MON-VER poll.
     * @param cls       Message class byte.
     * @param id        Message ID byte.
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Total frame length in bytes, or 0 if out is too small.
     */
    inline uint16_t buildPoll(uint8_t cls, uint8_t id,
                              uint8_t* out, uint16_t outBufLen)
    {
        return buildFrame(cls, id, nullptr, 0U, out, outBufLen);
    }

    /**
     * @brief Builds a UBX-CFG-PRT frame to configure UART1 protocol masks and baud rate.
     * @details Legacy command for M6 / M7 / M8 modules (protocol <= 23.01).
     *          The UART framing mode is fixed at 8N1 (UBX_UART_MODE_8N1).
     * @param portId    Port identifier (UBX_UART1_PORT_ID = 1).
     * @param baud      Desired baud rate in bits/s.
     * @param inProto   Input protocol mask (e.g. UBX_PROTO_MASK_UBX | UBX_PROTO_MASK_NMEA).
     * @param outProto  Output protocol mask (e.g. UBX_PROTO_MASK_UBX).
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Total frame length in bytes, or 0 if out is too small.
     */
    inline uint16_t buildCfgPrt(uint8_t portId, uint32_t baud,
                                uint16_t inProto, uint16_t outProto,
                                uint8_t* out, uint16_t outBufLen)
    {
        uint8_t payload[UBX_CFGPRT_PAYLOAD_LEN];
        memset(payload, 0, sizeof(payload));

        payload[0] = portId;
        detail::writeU4LE(&payload[4],  UBX_UART_MODE_8N1);
        detail::writeU4LE(&payload[8],  baud);
        detail::writeU2LE(&payload[12], inProto);
        detail::writeU2LE(&payload[14], outProto);

        return buildFrame(UBX_CLASS_CFG, UBX_ID_CFG_PRT,
                          payload, UBX_CFGPRT_PAYLOAD_LEN, out, outBufLen);
    }

    /**
     * @brief Builds a UBX-CFG-RATE frame to set the navigation measurement rate.
     * @details Legacy command for M6 / M7 / M8 modules (protocol <= 23.01).
     * @param measRateMs  Measurement period in milliseconds (e.g. 100 for 10 Hz).
     * @param navRate     Navigation solutions per measurement epoch (normally 1).
     * @param timeRef     Reference time system (0 = UTC).
     * @param out         Caller-supplied output buffer.
     * @param outBufLen   Size of the output buffer in bytes.
     * @return Total frame length in bytes, or 0 if out is too small.
     */
    inline uint16_t buildCfgRate(uint16_t measRateMs, uint16_t navRate,
                                 uint16_t timeRef,
                                 uint8_t* out, uint16_t outBufLen)
    {
        uint8_t payload[UBX_CFGRATE_PAYLOAD_LEN];
        memset(payload, 0, sizeof(payload));
        detail::writeU2LE(&payload[0], measRateMs);
        detail::writeU2LE(&payload[2], navRate);
        detail::writeU2LE(&payload[4], timeRef);

        return buildFrame(UBX_CLASS_CFG, UBX_ID_CFG_RATE,
                          payload, UBX_CFGRATE_PAYLOAD_LEN, out, outBufLen);
    }

    /**
     * @brief Builds a UBX-CFG-MSG frame to set the output rate of a single message.
     * @details Legacy command for M6 / M7 / M8 modules (protocol <= 23.01).
     *          Sets the message rate on the current port only.
     * @param msgClass  Message class of the message to configure.
     * @param msgId     Message ID of the message to configure.
     * @param rate      Output rate in navigation epochs (0 = disabled, 1 = every epoch).
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Total frame length in bytes, or 0 if out is too small.
     */
    inline uint16_t buildCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t rate,
                                uint8_t* out, uint16_t outBufLen)
    {
        uint8_t payload[UBX_CFGMSG_PAYLOAD_LEN];
        memset(payload, 0, sizeof(payload));
        payload[0] = msgClass;
        payload[1] = msgId;
        payload[2] = rate;

        return buildFrame(UBX_CLASS_CFG, UBX_ID_CFG_MSG,
                          payload, UBX_CFGMSG_PAYLOAD_LEN, out, outBufLen);
    }

    /**
     * @brief Builds a UBX-CFG-CFG frame to save the current configuration to all
     *        available non-volatile storage.
     * @details Legacy command for M6 / M7 / M8 modules (protocol <= 23.01).
     *          Sets clearMask and loadMask to zero and saveMask to UBX_CFGCFG_SAVE_ALL
     *          (bits 0–4), persisting all configuration subsections.
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Total frame length in bytes, or 0 if out is too small.
     */
    inline uint16_t buildCfgCfgSave(uint8_t* out, uint16_t outBufLen)
    {
        uint8_t payload[UBX_CFGCFG_PAYLOAD_LEN];
        memset(payload, 0, sizeof(payload));

        detail::writeU4LE(&payload[0], 0x00000000UL);          // clearMask
        detail::writeU4LE(&payload[4], UBX_CFGCFG_SAVE_ALL);   // saveMask
        detail::writeU4LE(&payload[8], 0x00000000UL);          // loadMask

        return buildFrame(UBX_CLASS_CFG, UBX_ID_CFG_CFG,
                          payload, UBX_CFGCFG_PAYLOAD_LEN, out, outBufLen);
    }

    /**
     * @brief Appends a uint8_t (1-byte) key-value pair to a CFG-VALSET cfgData buffer.
     * @details Build the cfgData buffer by chaining calls to appendKeyU1/U2/U4 before
     *          passing it to buildCfgValset().  Each call advances the write position
     *          by (4 + value_size) bytes.
     * @param buf Buffer receiving the concatenated key-value data.
     * @param pos Current write position within buf.
     * @param key 32-bit configuration key ID (UBXKEY_* constant).
     * @param val 1-byte value to associate with the key.
     * @return Updated write position after the appended pair (pos + 5).
     */
    inline uint16_t appendKeyU1(uint8_t* buf, uint16_t pos,
                                uint32_t key, uint8_t val)
    {
        detail::writeU4LE(&buf[pos], key);
        buf[pos + 4U] = val;
        return pos + 5U;
    }

    /**
     * @brief Appends a uint16_t (2-byte) key-value pair to a CFG-VALSET cfgData buffer.
     * @param buf Buffer receiving the concatenated key-value data.
     * @param pos Current write position within buf.
     * @param key 32-bit configuration key ID (UBXKEY_* constant).
     * @param val 2-byte little-endian value to associate with the key.
     * @return Updated write position after the appended pair (pos + 6).
     */
    inline uint16_t appendKeyU2(uint8_t* buf, uint16_t pos,
                                uint32_t key, uint16_t val)
    {
        detail::writeU4LE(&buf[pos], key);
        detail::writeU2LE(&buf[pos + 4U], val);
        return pos + 6U;
    }

    /**
     * @brief Appends a uint32_t (4-byte) key-value pair to a CFG-VALSET cfgData buffer.
     * @param buf Buffer receiving the concatenated key-value data.
     * @param pos Current write position within buf.
     * @param key 32-bit configuration key ID (UBXKEY_* constant).
     * @param val 4-byte little-endian value to associate with the key.
     * @return Updated write position after the appended pair (pos + 8).
     */
    inline uint16_t appendKeyU4(uint8_t* buf, uint16_t pos,
                                uint32_t key, uint32_t val)
    {
        detail::writeU4LE(&buf[pos], key);
        detail::writeU4LE(&buf[pos + 4U], val);
        return pos + 8U;
    }

    /**
     * @brief Builds a UBX-CFG-VALSET frame to write one or more configuration values.
     * @details New-style command for M9 / M10 modules (protocol >= 27).  Build the
     *          cfgData buffer by calling appendKeyU1(), appendKeyU2(), and appendKeyU4()
     *          before invoking this function.  The internal payload buffer is sized to
     *          UBX_CFGVALSET_MAX_KV_LEN bytes; cfgLen must not exceed this.
     * @param layers    Bitmask of target storage layers (UBX_VALSET_LAYER_RAM | BBR | FLASH).
     * @param cfgData   Concatenated key-value pairs produced by the appendKey* helpers.
     * @param cfgLen    Length of cfgData in bytes.
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Total frame length in bytes, or 0 if the payload overflows the internal
     *         buffer or out is too small.
     */
    inline uint16_t buildCfgValset(uint8_t layers,
                                   const uint8_t* cfgData, uint16_t cfgLen,
                                   uint8_t* out, uint16_t outBufLen)
    {
        const uint16_t payloadLen = UBX_CFGVAL_HDR_LEN + cfgLen;
        uint8_t payload[UBX_CFGVALSET_MAX_KV_LEN];
        if (payloadLen > sizeof(payload)) { return 0U; }

        payload[0] = 0x00U;  // version 0 (transactionless)
        payload[1] = layers;
        payload[2] = 0x00U;  // reserved
        payload[3] = 0x00U;  // reserved
        if (cfgLen > 0U) { memcpy(&payload[4], cfgData, cfgLen); }

        return buildFrame(UBX_CLASS_CFG, UBX_ID_CFG_VALSET,
                          payload, payloadLen, out, outBufLen);
    }

    /**
     * @brief Builds a UBX-CFG-VALGET frame to read configuration values from a layer.
     * @details New-style command for M9 / M10 modules (protocol >= 27).  The internal
     *          payload buffer is UBX_CFGVALGET_BUF_LEN bytes; after the 4-byte header
     *          this limits the request to (UBX_CFGVALGET_BUF_LEN - 4) / 4 = 7 key IDs.
     *          All callers in this library request at most 3 keys.
     * @param layer     Layer to read from (UBX_VALGET_LAYER_RAM, BBR, FLASH, or DEFAULT).
     * @param keys      Array of 32-bit key IDs to request.
     * @param numKeys   Number of entries in keys[].
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Total frame length in bytes, or 0 if the payload overflows the internal
     *         buffer or out is too small.
     */
    inline uint16_t buildCfgValget(uint8_t layer,
                                   const uint32_t* keys, uint8_t numKeys,
                                   uint8_t* out, uint16_t outBufLen)
    {
        const uint16_t payloadLen = 4U + (static_cast<uint16_t>(numKeys) * 4U);
        uint8_t payload[UBX_CFGVALGET_BUF_LEN];
        if (payloadLen > sizeof(payload)) { return 0U; }

        payload[0] = 0x00U;  // version
        payload[1] = layer;
        payload[2] = 0x00U;  // position LSB
        payload[3] = 0x00U;  // position MSB

        for (uint8_t i = 0U; i < numKeys; ++i) {
            detail::writeU4LE(&payload[4U + i * 4U], keys[i]);
        }

        return buildFrame(UBX_CLASS_CFG, UBX_ID_CFG_VALGET,
                          payload, payloadLen, out, outBufLen);
    }

    /**
     * @brief Builds an NMEA PUBX,41 sentence to configure a u-blox UART port.
     * @details The PUBX,41 sentence is processed by u-blox modules even when only
     *          NMEA input is enabled, making it the sole recovery path for re-enabling
     *          UBX binary I/O during the Phase 0 baud-rate sweep.  No ACK is returned
     *          for NMEA sentences; the caller infers success from subsequent UBX contact.
     *          Sentence format:
     *            $PUBX,41,<portId>,<inProto:04X>,<outProto:04X>,<baud>,0*<CS>\r\n
     * @param portId    UART port identifier (UBX_UART1_PORT_ID = 1).
     * @param inProto   Input protocol mask, formatted as four hex digits.
     * @param outProto  Output protocol mask, formatted as four hex digits.
     * @param baud      Baud rate to apply after the sentence is processed.
     * @param out       Caller-supplied output buffer.
     * @param outBufLen Size of the output buffer in bytes.
     * @return Sentence length including \r\n but excluding the snprintf null terminator,
     *         or 0 if the buffer is too small.
     */
    inline uint16_t buildPubx41(uint8_t portId,
                                uint16_t inProto, uint16_t outProto,
                                uint32_t baud,
                                uint8_t* out, uint16_t outBufLen)
    {
        char body[64];
        const int bodyLen = snprintf(body, sizeof(body),
                                     "PUBX,41,%u,%04X,%04X,%lu,0",
                                     static_cast<unsigned>(portId),
                                     static_cast<unsigned>(inProto),
                                     static_cast<unsigned>(outProto),
                                     static_cast<unsigned long>(baud));

        if ((bodyLen <= 0) || (bodyLen >= static_cast<int>(sizeof(body)))) {
            return 0U;
        }

        uint8_t cs = 0U;
        for (int i = 0; i < bodyLen; ++i) {
            cs ^= static_cast<uint8_t>(body[i]);
        }

        const int total = snprintf(reinterpret_cast<char*>(out), outBufLen,
                                   "$%s*%02X\r\n", body, static_cast<unsigned>(cs));

        if ((total <= 0) || (total >= static_cast<int>(outBufLen))) { return 0U; }
        return static_cast<uint16_t>(total);
    }

} // namespace UbxMessageBuilder
