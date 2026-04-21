//
//  FrequencyScanSample.swift
//  TinkerRocketApp
//
//  One sample of a base-station frequency scan: the channel centre frequency
//  and the instantaneous RSSI reported by the LoRa front end.
//

import Foundation

struct FrequencyScanSample: Identifiable, Equatable {
    let id = UUID()
    let freqMHz: Float
    let rssiDbm: Int
}
