//
//  SensorCalStatus.swift
//  TinkerRocketApp
//
//  Decoded form of the FC's SensorCalStatusData frame (issue #132): the
//  gyro zero-rate bias + high-g accel bias currently stored in the rocket's
//  NVS.  Arrives on the file_ops characteristic behind a 0xCB discriminator
//  (sibling of the 0xCA mag-cal frame).  Wire layout mirrors
//  SensorCalStatusData in RocketComputerTypes.h — keep both in sync.
//
//  Wire (19 bytes, little-endian, packed):
//    [0]      uint8  valid       (1 if a cal is stored)
//    [1..2]   int16  gyro_x      (raw gyro LSB)
//    [3..4]   int16  gyro_y
//    [5..6]   int16  gyro_z
//    [7..10]  float  hg_x        (high-g accel bias, m/s²)
//    [11..14] float  hg_y
//    [15..18] float  hg_z
//

import Foundation

struct SensorCalStatus: Equatable {
    let valid: Bool
    let gyroX: Int16
    let gyroY: Int16
    let gyroZ: Int16
    let hgX: Float
    let hgY: Float
    let hgZ: Float

    /// Decode the payload *after* the 0xCB discriminator byte.
    static func decode(_ bytes: [UInt8]) -> SensorCalStatus? {
        guard bytes.count >= 19 else { return nil }
        func i16(_ o: Int) -> Int16 {
            bytes.withUnsafeBufferPointer {
                UnsafeRawBufferPointer($0).loadUnaligned(fromByteOffset: o, as: Int16.self)
            }
        }
        func f32(_ o: Int) -> Float {
            bytes.withUnsafeBufferPointer {
                UnsafeRawBufferPointer($0).loadUnaligned(fromByteOffset: o, as: Float.self)
            }
        }
        return SensorCalStatus(
            valid: bytes[0] != 0,
            gyroX: i16(1), gyroY: i16(3), gyroZ: i16(5),
            hgX: f32(7), hgY: f32(11), hgZ: f32(15))
    }
}
