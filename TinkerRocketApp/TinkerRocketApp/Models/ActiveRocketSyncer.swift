//
//  ActiveRocketSyncer.swift
//  TinkerRocketApp
//
//  Pushes the active rocket profile to the connected flight computer
//  (issue #132).  The app is source-of-truth: on connect we push the whole
//  active profile so the rocket always flies the profile's settings, never
//  whatever happened to be in its NVS.  This is what fixes the
//  "flew the wrong rocket's settings" problem.
//
//  Lifecycle: the dashboard calls attach(device:store:) when a rocket
//  connects and detach() on disconnect.  While attached, the syncer also
//  re-pushes when the user switches the active profile, and surfaces a
//  mag-cal advisory (cal is board-specific — see MagCalData).
//
//  Why push the whole profile instead of diffing field-by-field: the config
//  readback doesn't echo every field (servo biases 2-4, roll waypoints, and
//  sounds aren't in it), so a reliable diff isn't possible.  Connects are
//  infrequent (once per flying session), so a handful of idempotent writes
//  is cheaper than the bugs a partial diff would invite.  Edits made while
//  disconnected are saved to the profile and ride out on the next connect —
//  that is the "offline queue" the design called for, for free.
//

import Foundation
import Combine

final class ActiveRocketSyncer: ObservableObject {

    // Workaround for swiftlang/swift#87316: with SWIFT_DEFAULT_ACTOR_ISOLATION
    // = MainActor, the implicit isolated deinit routes through the runtime's
    // back-deploy shim (swift_task_deinitOnExecutorMainActorBackDeploy), which
    // aborts with "pointer being freed was not allocated" when the object is
    // deallocated inside a synchronous XCTest. This class has no deinit-time
    // logic, so skipping the executor hop is free — and it un-crashes every
    // test that creates and tears down an instance.
    nonisolated deinit {}

    enum SyncState: Equatable {
        case idle              // not connected, or base station
        case awaitingSync      // attached to a rocket; config readback not in yet (#375)
        case noProfile         // connected to a rocket but no active profile
        case syncing
        case synced
        case failed(String)
    }

    /// Calibration is tied to a physical board, so it can't be blindly pushed.
    /// Shared by mag cal and sensor cal.
    enum CalAdvisory: Equatable {
        case none
        case missing                                   // no cal anywhere — run calibration
        case boardMismatch(savedOn: String, current: String)  // profile cal is for another board
        case rocketHasUnsavedCal                       // rocket carries a cal the profile doesn't
    }

    @Published private(set) var syncState: SyncState = .idle
    @Published private(set) var magCalAdvisory: CalAdvisory = .none
    @Published private(set) var sensorCalAdvisory: CalAdvisory = .none
    /// A profile previously flown on the just-connected board, offered as a
    /// soft switch suggestion.  The user always confirms — never auto-applied.
    @Published private(set) var suggestedProfileId: UUID?

    private weak var device: BLEDevice?
    private weak var store: RocketProfileStore?
    private var cancellables = Set<AnyCancellable>()
    private var magCalReadPending = false
    private var sensorCalReadPending = false

    // MARK: - Lifecycle

    func attach(device: BLEDevice, store: RocketProfileStore) {
        // Idempotent for the same device+store pair (#375): the dashboard now
        // re-attaches on every device-list / active-device change, and a
        // redundant call must not tear down subscriptions or reset a .synced
        // state back through the pipeline. A NEW device object (reconnects
        // create one) intentionally falls through to a full re-attach.
        if device === self.device && store === self.store { return }

        detach()
        self.device = device
        self.store = store

        guard !device.isBaseStation else {
            // Base station is a read-only display of the active rocket; it
            // never receives a profile push.
            syncState = .idle
            return
        }

        // Visible from the first moment we're responsible for this rocket:
        // "connected but not yet pushed" must not render as silent .idle
        // (#375 — that silence is how an unsynced rocket reached the pad).
        syncState = .awaitingSync

        // Sync once, when the first config readback AND the hardware id are
        // both in hand.  unitID arrives in a later readback message than the
        // main config, and we need it for the mag-cal board check + the
        // last-used binding, so wait for both.
        device.$rocketConfig.combineLatest(device.$unitID)
            .filter { cfg, uid in cfg != nil && !uid.isEmpty }
            .first()
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _ in self?.onReadyToSync() }
            .store(in: &cancellables)

        // Connect-time cal READ replies (only honoured right after we ask).
        device.$magCalStatus
            .compactMap { $0 }
            .receive(on: DispatchQueue.main)
            .sink { [weak self] status in self?.handleMagCalStatus(status) }
            .store(in: &cancellables)

        device.$sensorCalStatus
            .compactMap { $0 }
            .receive(on: DispatchQueue.main)
            .sink { [weak self] status in self?.handleSensorCalStatus(status) }
            .store(in: &cancellables)

        // Re-push when the user switches the active profile mid-connection.
        store.$activeId
            .removeDuplicates()
            .dropFirst()
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _ in self?.handleActiveProfileSwitch() }
            .store(in: &cancellables)
    }

    func detach() {
        cancellables.removeAll()
        device = nil
        store = nil
        magCalReadPending = false
        sensorCalReadPending = false
        syncState = .idle
        magCalAdvisory = .none
        sensorCalAdvisory = .none
        suggestedProfileId = nil
    }

    // MARK: - Triggers

    private func onReadyToSync() {
        guard let device, !device.isBaseStation else { return }
        computeSuggestion()
        performSync()
    }

    private func handleActiveProfileSwitch() {
        guard let device, device.isConnected, !device.isBaseStation else { return }
        suggestedProfileId = nil   // user has chosen; drop the hint
        performSync()
    }

    // MARK: - Sync

    private func performSync() {
        guard let device, let store, device.isConnected else {
            syncState = .idle
            return
        }
        guard let profile = store.activeProfile else {
            syncState = .noProfile
            return
        }

        syncState = .syncing

        device.sendServoConfig(
            biases: [profile.servoBias1, profile.servoBias2,
                     profile.servoBias3, profile.servoBias4],
            hz: profile.servoHz, minUs: profile.servoMinUs, maxUs: profile.servoMaxUs,
            finMinDeg: profile.finMinDeg, finMaxDeg: profile.finMaxDeg)
        device.sendPIDConfig(
            kp: profile.pidKp, ki: profile.pidKi, kd: profile.pidKd,
            minCmd: profile.pidMinCmd, maxCmd: profile.pidMaxCmd)
        device.sendServoControlConfig(enabled: profile.servoControlEnabled)
        device.sendGainScheduleConfig(enabled: profile.gainScheduleEnabled)
        device.sendRollControlConfig(useAngleControl: profile.useAngleControl,
                                     rollDelayMs: profile.rollDelayMs,
                                     rateCapDps: profile.rateCapDps,
                                     kpAngle: profile.kpAngle,
                                     integralSepThreshold: profile.integralSepThreshold)
        // mode byte is a legacy wire field (pre-v4 firmware); always send .angle
        device.sendRollProfile(waypoints: profile.rollWaypoints.map {
            (time: $0.timeSeconds, angle: $0.angleDeg, mode: RollSegmentMode.angle.rawValue)
        })
        device.sendGuidanceConfig(enabled: profile.guidanceEnabled,
                                  navGain: profile.pnNavGain, maxAccel: profile.pnMaxAccel,
                                  accelToFin: profile.pnAccelToFin, maxFinDeg: profile.pnMaxFinDeg,
                                  minSpeed: profile.pnMinSpeed, coastDelayMs: profile.pnCoastDelayMs,
                                  targetMode: profile.pnTargetMode,
                                  targetE: profile.pnTargetE, targetN: profile.pnTargetN,
                                  targetAlt: profile.pnTargetAltM)
        device.sendFinConfig(ringMode: profile.finRingMode,
                             servoAtSlot: profile.finServoAtSlot,
                             reverse: profile.finReverse,
                             rollReverse: profile.finRollReverse)
        device.sendCameraConfig(cameraType: profile.cameraType)
        device.sendImuOrientationConfig(profile.imuOrientSetting)
        device.sendSoundConfig(enabled: profile.soundsEnabled)
        device.sendPyroConfig(channels: [
            (profile.pyro1Enabled, profile.pyro1TriggerMode, profile.pyro1TriggerValue),
            (profile.pyro2Enabled, profile.pyro2TriggerMode, profile.pyro2TriggerValue),
            (profile.pyro3Enabled, profile.pyro3TriggerMode, profile.pyro3TriggerValue),
            (profile.pyro4Enabled, profile.pyro4TriggerMode, profile.pyro4TriggerValue),
        ])

        syncMagCal(profile: profile, device: device)
        syncSensorCal(profile: profile, device: device)

        // Record which board this profile last flew on (soft pre-select).
        store.update(profile.id) { $0.lastUsedUnitID = device.unitID }

        // Optimistic, like the rest of the app's config writes (no per-command
        // ack on this link).  Hold "syncing" briefly so the badge is visible.
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.8) { [weak self] in
            guard let self, case .syncing = self.syncState else { return }
            self.syncState = .synced
        }
    }

    // MARK: - Mag cal

    /// What to do with mag cal on connect, given the active profile's cal and
    /// the connected board's id.  Pure so it can be unit-tested without a
    /// peripheral (mirrors BLEDevice.autoApplyRefusalReason).
    enum MagCalSyncAction: Equatable {
        case push(MagCalData)                           // profile cal matches this board
        case warnMismatch(savedOn: String, current: String)  // cal is for another board
        case readRocket                                 // profile has none — query the rocket
    }

    static func magCalSyncAction(profileCal: MagCalData?,
                                 deviceUnitID: String) -> MagCalSyncAction {
        guard let cal = profileCal else { return .readRocket }
        if cal.calibratedOnUnitID == deviceUnitID {
            return .push(cal)
        }
        return .warnMismatch(savedOn: cal.calibratedOnUnitID, current: deviceUnitID)
    }

    private func syncMagCal(profile: RocketProfile, device: BLEDevice) {
        switch Self.magCalSyncAction(profileCal: profile.magCal,
                                     deviceUnitID: device.unitID) {
        case .push(let cal):
            magCalAdvisory = .none
            device.sendMagCalApply(cx: cal.offsetX, cy: cal.offsetY, cz: cal.offsetZ,
                                   fieldR_uT: cal.fieldR_uT, residualUT: cal.residualUT)
        case .warnMismatch(let savedOn, let current):
            magCalAdvisory = .boardMismatch(savedOn: savedOn, current: current)
        case .readRocket:
            // Ask the rocket whether it carries a cal so we can offer to import
            // it (rather than silently overwriting either side).
            magCalReadPending = true
            device.sendMagCalRead()
        }
    }

    private func handleMagCalStatus(_ status: MagCalStatus) {
        guard magCalReadPending else { return }   // only our connect-time READ
        magCalReadPending = false
        magCalAdvisory = (status.subType == .applied) ? .rocketHasUnsavedCal : .missing
    }

    /// Import the rocket's currently-applied cal into the active profile, tagged
    /// with this board's id.  Called from the UI when the user accepts the
    /// `rocketHasUnsavedCal` advisory.
    func importRocketCalIntoActiveProfile() {
        guard let device, let store,
              let profile = store.activeProfile,
              let status = device.magCalStatus, status.subType == .applied
        else { return }
        store.update(profile.id) {
            $0.magCal = MagCalData(status: status, unitID: device.unitID)
        }
        magCalAdvisory = .none
    }

    // MARK: - Sensor cal (gyro + high-g) — mirrors mag cal

    enum SensorCalSyncAction: Equatable {
        case push(SensorCalData)
        case warnMismatch(savedOn: String, current: String)
        case readRocket
    }

    static func sensorCalSyncAction(profileCal: SensorCalData?,
                                    deviceUnitID: String) -> SensorCalSyncAction {
        guard let cal = profileCal else { return .readRocket }
        if cal.calibratedOnUnitID == deviceUnitID {
            return .push(cal)
        }
        return .warnMismatch(savedOn: cal.calibratedOnUnitID, current: deviceUnitID)
    }

    private func syncSensorCal(profile: RocketProfile, device: BLEDevice) {
        switch Self.sensorCalSyncAction(profileCal: profile.sensorCal,
                                        deviceUnitID: device.unitID) {
        case .push(let cal):
            sensorCalAdvisory = .none
            device.sendSensorCalApply(gyroX: cal.gyroX, gyroY: cal.gyroY, gyroZ: cal.gyroZ,
                                      hgX: cal.hgX, hgY: cal.hgY, hgZ: cal.hgZ)
        case .warnMismatch(let savedOn, let current):
            sensorCalAdvisory = .boardMismatch(savedOn: savedOn, current: current)
        case .readRocket:
            sensorCalReadPending = true
            device.sendSensorCalRead()
        }
    }

    private func handleSensorCalStatus(_ status: SensorCalStatus) {
        guard sensorCalReadPending else { return }
        sensorCalReadPending = false
        sensorCalAdvisory = status.valid ? .rocketHasUnsavedCal : .missing
    }

    func importRocketSensorCalIntoActiveProfile() {
        guard let device, let store,
              let profile = store.activeProfile,
              let status = device.sensorCalStatus, status.valid
        else { return }
        store.update(profile.id) {
            $0.sensorCal = SensorCalData(status: status, unitID: device.unitID)
        }
        sensorCalAdvisory = .none
    }

    // MARK: - Suggestion

    private func computeSuggestion() {
        guard let device, let store else { return }
        suggestedProfileId = Self.suggestedProfile(
            in: store.profiles, active: store.activeId, unitID: device.unitID)
    }

    /// A profile previously flown on `unitID` other than the active one, or
    /// nil if none.  Pure for testability.
    static func suggestedProfile(in profiles: [RocketProfile],
                                 active: UUID?, unitID: String) -> UUID? {
        guard !unitID.isEmpty else { return nil }
        return profiles.first { $0.lastUsedUnitID == unitID && $0.id != active }?.id
    }
}

extension MagCalData {
    /// Snapshot a cal status frame into a savable profile cal, tagged with the
    /// board it was captured on.
    init(status: MagCalStatus, unitID: String) {
        self.init(offsetX: status.offsetX,
                  offsetY: status.offsetY,
                  offsetZ: status.offsetZ,
                  fieldR_uT: status.fieldR_uT,
                  residualUT: status.residualUT,
                  calibratedOnUnitID: unitID,
                  calibratedAt: Date())
    }
}

extension SensorCalData {
    /// Snapshot a sensor cal readback into a savable profile cal, tagged with
    /// the board it was captured on.
    init(status: SensorCalStatus, unitID: String) {
        self.init(gyroX: status.gyroX,
                  gyroY: status.gyroY,
                  gyroZ: status.gyroZ,
                  hgX: status.hgX,
                  hgY: status.hgY,
                  hgZ: status.hgZ,
                  calibratedOnUnitID: unitID,
                  calibratedAt: Date())
    }
}
