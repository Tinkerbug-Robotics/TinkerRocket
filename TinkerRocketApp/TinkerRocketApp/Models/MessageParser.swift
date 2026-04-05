//
//  MessageParser.swift
//  TinkerRocketApp
//
//  Created by Claude Code
//  Parse binary log files containing framed sensor messages
//

import Foundation

// MARK: - Message Frame

nonisolated struct MessageFrame {
    let type: UInt8
    let payload: Data
    let timestamp: UInt32  // extracted from first 4 bytes of payload (time_us)
}

// MARK: - Message Parser

/// nonisolated: MessageParser is used by CSVGenerator on a background queue.
nonisolated class MessageParser {

    // Frame format: [0xAA 0x55 0xAA 0x55][Type][Length][Payload][CRC16_MSB][CRC16_LSB]
    private static let PREAMBLE: [UInt8] = [0xAA, 0x55, 0xAA, 0x55]
    private static let PREAMBLE_SIZE = 4
    private static let MIN_FRAME_SIZE = 4 + 1 + 1 + 2  // preamble + type + length + crc

    /// Parse a binary log file and extract all valid message frames
    func parseLogFile(_ url: URL) throws -> [MessageFrame] {
        let data = try Data(contentsOf: url)
        var frames: [MessageFrame] = []
        var index = 0

        while index < data.count {
            // 1. Find preamble
            guard let preambleIndex = findPreamble(in: data, startingAt: index) else {
                // No more preambles found
                break
            }

            index = preambleIndex + MessageParser.PREAMBLE_SIZE

            // 2. Check if we have enough data for type, length, and CRC
            guard index + 3 < data.count else {
                break
            }

            // 3. Extract type and length
            let type = data[index]
            let length = Int(data[index + 1])
            index += 2

            // 4. Validate frame size
            let expectedFrameEnd = index + length + 2  // payload + CRC
            guard expectedFrameEnd <= data.count else {
                // Incomplete frame at end of data — stop parsing
                break
            }

            // 5. Extract payload
            let payload = data.subdata(in: index..<(index + length))
            index += length

            // 6. Extract and verify CRC
            let crcMSB = data[index]
            let crcLSB = data[index + 1]
            let receivedCRC = (UInt16(crcMSB) << 8) | UInt16(crcLSB)
            index += 2

            // Calculate CRC over Type + Length + Payload
            var crcData = Data()
            crcData.append(type)
            crcData.append(UInt8(length))
            crcData.append(payload)

            let calculatedCRC = calculateCRC16(crcData)

            if calculatedCRC != receivedCRC {
                // CRC mismatch, skip this frame
                print("CRC mismatch: expected \(String(format: "0x%04X", receivedCRC)), got \(String(format: "0x%04X", calculatedCRC))")
                continue
            }

            // 7. Extract timestamp from payload (first 4 bytes)
            let timestamp: UInt32
            if payload.count >= 4 {
                timestamp = payload.withUnsafeBytes { $0.load(as: UInt32.self).littleEndian }
            } else {
                timestamp = 0
            }

            // 8. Create and store frame
            let frame = MessageFrame(type: type, payload: payload, timestamp: timestamp)
            frames.append(frame)
        }

        return frames
    }

    /// Find the next preamble [0xAA 0x55 0xAA 0x55] starting at the given index
    private func findPreamble(in data: Data, startingAt: Int) -> Int? {
        let preamble = MessageParser.PREAMBLE
        var index = startingAt

        while index <= data.count - preamble.count {
            var match = true
            for i in 0..<preamble.count {
                if data[index + i] != preamble[i] {
                    match = false
                    break
                }
            }
            if match {
                return index
            }
            index += 1
        }

        return nil
    }

    /// Calculate CRC16 using polynomial 0x8001, initial 0x0000
    /// This matches the C++ implementation in libraries/CRC/src/CRC16.cpp
    private func calculateCRC16(_ data: Data) -> UInt16 {
        let polynome: UInt16 = 0x8001
        var crc: UInt16 = 0x0000

        for byte in data {
            // XOR CRC with (byte << 8)
            crc ^= UInt16(byte) << 8

            // Process 8 bits
            for _ in 0..<8 {
                if (crc & 0x8000) != 0 {
                    // MSB is 1: shift left and XOR with polynome
                    crc <<= 1
                    crc ^= polynome
                } else {
                    // MSB is 0: just shift left
                    crc <<= 1
                }
            }
        }

        return crc
    }
}
