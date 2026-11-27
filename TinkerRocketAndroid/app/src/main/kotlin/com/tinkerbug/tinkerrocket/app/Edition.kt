package com.tinkerbug.tinkerrocket.app

import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import com.tinkerbug.tinkerrocket.BuildConfig
import com.tinkerbug.tinkerrocket.protocol.AppEdition
import com.tinkerbug.tinkerrocket.protocol.AppFeatures
import com.tinkerbug.tinkerrocket.protocol.BoardFamily
import com.tinkerbug.tinkerrocket.session.DeviceIdentity
import com.tinkerbug.tinkerrocket.session.DeviceSession

/** Which app this build is: the `store` flavor sets TR_STORE (AppEdition.kt, docs/app-editions.md). */
val currentEdition: AppEdition = if (BuildConfig.TR_STORE) AppEdition.STORE else AppEdition.DEV

/**
 * The rocket's board family, from what this link has heard. Null on a base
 * station: its own firmware carries a board suffix too ("-v5" on a
 * Tinker-Base V2), which names a base-station board, not a rocket's.
 */
fun boardFamilyOf(isBaseStation: Boolean, identity: DeviceIdentity): BoardFamily? =
    if (isBaseStation) {
        null
    } else {
        BoardFamily.of(
            identity.fcBoardRev, identity.ocBoardRev,
            identity.fcFirmwareVersion, identity.firmwareVersion,
        )
    }

/** What this build shows for the rocket [session] talks to (iOS `BLEDevice.features`). */
@Composable
fun rememberFeatures(session: DeviceSession?): AppFeatures {
    if (session == null) return AppFeatures(currentEdition, null)
    val identity by session.identity.collectAsState()
    return AppFeatures(currentEdition, boardFamilyOf(session.isBaseStation, identity))
}
