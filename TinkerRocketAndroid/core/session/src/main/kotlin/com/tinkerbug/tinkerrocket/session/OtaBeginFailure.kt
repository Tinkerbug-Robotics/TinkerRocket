package com.tinkerbug.tinkerrocket.session

/**
 * What the update says when OTA_BEGIN gets no answer in time.  iOS twin:
 * OTABeginFailure.swift.
 *
 * The firmware screen stays reachable while the flight computer is booting,
 * stuck or in rescue mode, and offers the flight computer as a target the
 * whole time, so this is where the operator learns whether the flight computer
 * answered.  On the relay path "the device" is ambiguous (the out computer
 * always answers), and a stuck flight computer is exactly the case worth
 * spelling out: it serves updates from its main loop or from rescue mode,
 * never from partway through setup.
 */
public object OtaBeginFailure {
    public fun notAccepted(targetIsFc: Boolean, timeoutS: Long): String {
        if (!targetIsFc) return "Device did not accept OTA_BEGIN within ${timeoutS}s"
        return "The flight computer did not answer within ${timeoutS}s. It takes updates once it is " +
            "running, or in rescue mode, but not while it is still starting up. Firmware " +
            "with rescue mode switches to it about two minutes after a stalled start; try " +
            "again then. Older firmware needs a USB flash."
    }
}
