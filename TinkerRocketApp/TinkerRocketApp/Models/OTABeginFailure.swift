//
//  OTABeginFailure.swift
//  TinkerRocketApp
//
//  What the update says when OTA_BEGIN gets no answer in time.  Android twin:
//  OtaBeginFailure in core/session.
//
//  The firmware screen stays reachable while the flight computer is booting,
//  stuck or in rescue mode (SettingsAccess.swift), and offers the flight
//  computer as a target the whole time, so this is where the operator learns
//  whether the flight computer answered.  On the relay path "the device" is
//  ambiguous (the out computer always answers), and a stuck flight computer is
//  exactly the case worth spelling out: it serves updates from its main loop
//  or from rescue mode, never from partway through setup.
//

import Foundation

enum OTABeginFailure {
    static func notAccepted(targetIsFC: Bool, timeoutS: TimeInterval) -> String {
        let s = Int(timeoutS)
        guard targetIsFC else { return "Device did not accept OTA_BEGIN within \(s)s" }
        return "The flight computer did not answer within \(s)s. It takes updates once it is "
            + "running, or in rescue mode, but not while it is still starting up. Firmware "
            + "with rescue mode switches to it about two minutes after a stalled start; try "
            + "again then. Older firmware needs a USB flash."
    }
}
