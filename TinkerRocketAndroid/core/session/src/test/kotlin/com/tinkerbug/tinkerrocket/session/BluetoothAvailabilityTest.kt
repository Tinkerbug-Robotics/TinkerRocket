package com.tinkerbug.tinkerrocket.session

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #1413. The states themselves need a phone with its radio off or its grant
 * revoked, which no unit test can arrange — so what is pinned here is the part
 * that can be: which state a set of facts means, what gets said about it, and
 * which of the two system switches to offer.
 */
class BluetoothAvailabilityTest {

    @Test
    fun `a missing adapter outranks everything else`() {
        // Nothing to permit and nothing to switch on, so neither of the other
        // two answers would be actionable.
        assertEquals(
            BluetoothAvailability.NO_ADAPTER,
            BluetoothAvailability.of(hasAdapter = false, adapterOn = false, permissionsGranted = false),
        )
        assertEquals(
            BluetoothAvailability.NO_ADAPTER,
            BluetoothAvailability.of(hasAdapter = false, adapterOn = true, permissionsGranted = true),
        )
    }

    @Test
    fun `a denied grant outranks the adapter state`() {
        // On API 31+ the adapter cannot be read reliably without the grant, so
        // reporting "off" here would be a guess presented as a fact.
        assertEquals(
            BluetoothAvailability.PERMISSION_DENIED,
            BluetoothAvailability.of(hasAdapter = true, adapterOn = true, permissionsGranted = false),
        )
        assertEquals(
            BluetoothAvailability.PERMISSION_DENIED,
            BluetoothAvailability.of(hasAdapter = true, adapterOn = false, permissionsGranted = false),
        )
    }

    @Test
    fun `an adapter that is present and permitted but off says so`() {
        assertEquals(
            BluetoothAvailability.ADAPTER_OFF,
            BluetoothAvailability.of(hasAdapter = true, adapterOn = false, permissionsGranted = true),
        )
    }

    @Test
    fun `everything in order is ready`() {
        assertEquals(
            BluetoothAvailability.READY,
            BluetoothAvailability.of(hasAdapter = true, adapterOn = true, permissionsGranted = true),
        )
    }

    @Test
    fun `only the actionable states give advice`() {
        assertNotNull(BluetoothAvailability.ADAPTER_OFF.advice)
        assertNotNull(BluetoothAvailability.PERMISSION_DENIED.advice)
        assertNull(BluetoothAvailability.READY.advice)
        assertNull(BluetoothAvailability.UNKNOWN.advice)
        assertNull(
            BluetoothAvailability.NO_ADAPTER.advice,
            "there is no fix for the reader to apply, so say nothing",
        )
    }

    @Test
    fun `each blocked state offers the switch that actually fixes it`() {
        // The adapter needs the system dialog; the grant needs the app's own
        // settings page. Sending either to the other's destination is a dead
        // end dressed up as a fix.
        assertEquals(BluetoothFix.ENABLE_BLUETOOTH, BluetoothAvailability.ADAPTER_OFF.fix)
        assertEquals(BluetoothFix.APP_SETTINGS, BluetoothAvailability.PERMISSION_DENIED.fix)
        assertEquals(BluetoothFix.NONE, BluetoothAvailability.READY.fix)
        assertEquals(BluetoothFix.NONE, BluetoothAvailability.NO_ADAPTER.fix)
        assertEquals(BluetoothFix.NONE, BluetoothAvailability.UNKNOWN.fix)
    }

    @Test
    fun `a state with advice always has a button, and one without never does`() {
        for (state in BluetoothAvailability.entries) {
            if (state.advice == null) {
                assertEquals(BluetoothFix.NONE, state.fix, "$state says nothing, so must offer nothing")
            } else {
                assertNotNull(state.fix.label, "$state gives advice, so must offer the switch for it")
            }
        }
    }

    @Test
    fun `only ready can scan`() {
        assertTrue(BluetoothAvailability.READY.canScan)
        for (state in BluetoothAvailability.entries - BluetoothAvailability.READY) {
            assertFalse(state.canScan, "$state must not start a scan that cannot run")
        }
    }

    @Test
    fun `every state has a headline`() {
        for (state in BluetoothAvailability.entries) {
            assertTrue(state.headline.isNotBlank(), "$state needs something beside the dot")
        }
        // The pill shows this instead of the stale scanner message when
        // Bluetooth is the thing that is wrong.
        assertEquals("Bluetooth is off", BluetoothAvailability.ADAPTER_OFF.headline)
    }
}
