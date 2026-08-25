//
//  ActiveRocketSyncer.swift
//  TinkerRocketApp
//
//  Reconciles the connected rocket's own settings with the app's profile
//  for that airframe (issues #132, #915).  The ROCKET is source-of-truth on
//  connect: it reports what it has, the app adopts it into the profile bound
//  to that board, and nothing is written to the rocket unless the user asks.
//
//  Why this way round (#915 reverses #132): the rocket persists essentially
//  its whole config in NVS and reloads it at boot, so it already remembers
//  what it was configured as.  The hazard #132 was aiming at — flying the
//  wrong rocket's settings — is better solved by binding a profile to a
//  board's hardware id than by having the phone overwrite whatever it
//  connects to.  Under the old rule, connecting to a rocket while the phone
//  held a different airframe's profile silently reconfigured the vehicle:
//  fourteen config frames inside two seconds, no user action, no UI notice.
//
//  Lifecycle: the dashboard calls attach(device:store:) when a rocket
//  connects and detach() on disconnect.  While attached, the syncer pushes
//  the whole profile only on an explicit user act — switching the active
//  profile onto this board, or pushProfileToRocket() from the UI — and
//  surfaces the cal advisories (cal is board-specific — see MagCalData).
//
//  What the app still can't see: the config readback echoes about half the
//  editable surface.  Servo trim 2-4, fin travel, fin layout, roll
//  waypoints, the PN guidance parameters and sounds are not reported, so
//  the settings screen shows the profile's values for those and cannot
//  verify them against the rocket.  `unreportedGroups` names them for the
//  UI; closing that gap needs a firmware-side full-config report.
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
        case syncing           // an explicit push of the whole profile is in flight
        case synced            // profile and rocket agree on everything reported
        /// The rocket reported settings the profile disagreed with; the
        /// profile was updated to match and these groups changed (#915).
        /// Informational, not a fault — the rocket kept flying what it had.
        case adopted([String])
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
    /// Since #915 binds the board's profile automatically this is normally
    /// nil (the bound profile IS the active one, and this excludes the active
    /// id); it still fires if a second profile claims the same board.
    @Published private(set) var suggestedProfileId: UUID?

    /// Name of a profile this syncer created for a board it had never seen,
    /// seeded from the rocket's own reported settings (#915).  Surfaced as an
    /// info line so a silently-created profile is never a surprise; cleared
    /// on detach.
    @Published private(set) var createdProfileName: String?

    /// Setting groups the CONNECTED rocket does not report back, so the app
    /// shows the profile's values for them and cannot verify them against the
    /// vehicle.  Empty once the firmware's config report has landed; the full
    /// pre-report list on older firmware, and the servo/fin/guidance groups on
    /// a mini, which has none of that hardware.
    @Published private(set) var unreportedGroups: [String] = []

    private weak var device: BLEDevice?
    private weak var store: RocketProfileStore?
    private var cancellables = Set<AnyCancellable>()
    private var magCalReadPending = false
    private var sensorCalReadPending = false
    /// The orientation setting rides a LATER readback frame than the main
    /// config (the OC queues imu_orient after config_identity), so the first
    /// adoption pass usually runs without it.  Armed once per attach to
    /// re-adopt when it lands; never re-armed, so a later user edit isn't
    /// mistaken for a rocket report.
    private var orientAdoptArmed = false
    /// Armed once per attach, for the same reason as orientAdoptArmed: the
    /// #915 config-report frames arrive after the main config readback.
    private var extrasAdoptArmed = false
    /// The profile id THIS syncer made active while binding the board.  The
    /// active-profile subscription is delivered on a later main-queue turn,
    /// so it cannot tell our own bind from a user tap — without this, binding
    /// a board would fire handleActiveProfileSwitch() and push the profile
    /// straight back at the rocket, reinstating the exact #915 behaviour the
    /// bind exists to remove.  Consumed by the first matching callback.
    private var selfSelectedProfileId: UUID?

    /// Which role the current attach ran under.  A renamed device can
    /// mis-parse its type from the BLE name until config_identity lands
    /// (e.g. a rocket called "SUBSONIC" reads as a base station via the
    /// legacy substring check), so the role at attach time isn't final —
    /// see the re-evaluation in attach().
    private var attachedAsBaseStation = false

    // MARK: - Lifecycle

    func attach(device: BLEDevice, store: RocketProfileStore) {
        // Idempotent for the same device+store pair (#375): the dashboard
        // re-attaches on every device-list / active-device change, and a
        // redundant call must not tear down subscriptions or reset a .synced
        // state back through the pipeline. A NEW device object (reconnects
        // create one) intentionally falls through to a full re-attach.
        //
        // Exception: if the device's ROLE changed since we attached (the
        // config_identity readback corrected a name-based mis-parse), fall
        // through and re-attach under the right role — otherwise a rocket
        // first mis-read as a base station keeps profile sync silently
        // disabled for the whole session (the #375 failure mode, back
        // through a different door).
        if device === self.device && store === self.store
            && attachedAsBaseStation == device.isBaseStation { return }

        detach()
        self.device = device
        self.store = store
        attachedAsBaseStation = device.isBaseStation

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
        attachedAsBaseStation = false
        magCalReadPending = false
        sensorCalReadPending = false
        orientAdoptArmed = false
        extrasAdoptArmed = false
        selfSelectedProfileId = nil
        syncState = .idle
        magCalAdvisory = .none
        sensorCalAdvisory = .none
        suggestedProfileId = nil
        createdProfileName = nil
        unreportedGroups = []
    }

    // MARK: - Triggers

    private func onReadyToSync() {
        guard let device, !device.isBaseStation else { return }
        bindProfileToBoard()
        computeSuggestion()
        adoptRocketConfig()
    }

    private func handleActiveProfileSwitch() {
        guard let device, let store, device.isConnected, !device.isBaseStation else { return }
        // Our own bind, arriving late — not a user choice, so no push.
        if let mine = selfSelectedProfileId, store.activeId == mine {
            selfSelectedProfileId = nil
            return
        }
        selfSelectedProfileId = nil
        suggestedProfileId = nil   // user has chosen; drop the hint
        createdProfileName = nil
        // Choosing a different profile for a CONNECTED rocket is the explicit
        // "make this rocket fly these settings" act, so it still pushes the
        // whole profile — the one place #915 leaves the old behaviour intact.
        pushProfileToRocket()
    }

    // MARK: - Binding

    /// Make the profile bound to the connected board the active one, creating
    /// one from the rocket's own settings if this board is new to the app.
    ///
    /// `lastUsedUnitID` was already a per-board binding; before #915 it only
    /// drove a suggestion the user had to accept while the *active* profile
    /// was pushed regardless.  Reading the binding instead of ignoring it is
    /// what lets the phone connect to any rocket in the field without
    /// changing it.
    private func bindProfileToBoard() {
        guard let device, let store, !device.unitID.isEmpty else { return }

        if let bound = store.profiles.first(where: { $0.lastUsedUnitID == device.unitID }) {
            if store.activeId != bound.id {
                selfSelectedProfileId = bound.id
                store.setActive(bound.id)
            }
            return
        }

        // Board we have never seen: adopt it as its own profile.  Pushing the
        // active profile here instead would be exactly the #915 failure —
        // connect to an unfamiliar rocket, reconfigure it.
        let fallback = "Rocket \(device.unitID.suffix(4))"
        let created = store.add(name: device.unitName.isEmpty ? fallback : device.unitName)
        store.update(created.id) { $0.lastUsedUnitID = device.unitID }
        selfSelectedProfileId = created.id
        store.setActive(created.id)
        createdProfileName = store.profiles.first { $0.id == created.id }?.name
    }

    // MARK: - Adoption (rocket wins)

    /// Take the rocket's reported settings into the bound profile.  Nothing is
    /// written to the rocket.
    private func adoptRocketConfig() {
        guard let device, let store, device.isConnected else {
            syncState = .idle
            return
        }
        guard let profile = store.activeProfile else {
            syncState = .noProfile
            return
        }
        guard let cfg = device.rocketConfig else {
            syncState = .awaitingSync
            return
        }

        unreportedGroups = cfg.unreportedGroups

        var probe = profile
        let changed = Self.adopt(&probe, from: cfg)
        if !changed.isEmpty { store.update(profile.id) { $0 = probe } }

        // A profile we just created from this same readback matches the rocket
        // by construction — reporting its every factory-default field as
        // "changed" would be noise.  The createdProfileName line covers it.
        syncState = (changed.isEmpty || createdProfileName != nil)
            ? .synced : .adopted(changed)

        armOrientationAdoptIfNeeded(device: device, cfg: cfg)
        armExtrasAdoptIfNeeded(device: device, cfg: cfg)

        syncMagCal(profile: profile, device: device)
        syncSensorCal(profile: profile, device: device)
    }

    /// The orientation setting arrives in a later readback frame than the main
    /// config, so adopt it again when it lands.  Only while it is still
    /// missing, only once, and only on firmware that reports it at all
    /// (pre-v3-orientation FCs never send the frame — the subscription simply
    /// never fires, and the profile keeps its own value).
    private func armOrientationAdoptIfNeeded(device: BLEDevice, cfg: RocketConfig) {
        guard cfg.imuOrientSetting == nil, !orientAdoptArmed else { return }
        orientAdoptArmed = true
        device.$rocketConfig
            .compactMap { $0?.imuOrientSetting }
            .first()
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _ in self?.adoptOrientationOnly() }
            .store(in: &cancellables)
    }

    /// The three config-report frames land after the main config, in their own
    /// paced readback slots.  Re-adopt as they arrive so the profile picks up
    /// the fin layout and guidance parameters without a reconnect.  Bounded to
    /// the connect burst by `extrasAdoptArmed`, so a later user edit — which
    /// also flows back through the report — isn't reported as the rocket
    /// disagreeing with the phone.
    private func armExtrasAdoptIfNeeded(device: BLEDevice, cfg: RocketConfig) {
        guard !cfg.unreportedGroups.isEmpty, !extrasAdoptArmed else { return }
        extrasAdoptArmed = true
        device.$rocketConfig
            .compactMap { $0 }
            .filter { $0.unreportedGroups.isEmpty }
            .first()
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _ in self?.adoptRocketConfig() }
            .store(in: &cancellables)
    }

    private func adoptOrientationOnly() {
        guard let device, let store, device.isConnected,
              let profile = store.activeProfile,
              let setting = device.rocketConfig?.imuOrientSetting,
              setting != profile.imuOrientSetting
        else { return }
        store.update(profile.id) { $0.imuOrientSetting = setting }
        guard createdProfileName == nil else { return }
        var groups: [String] = []
        if case .adopted(let existing) = syncState { groups = existing }
        if !groups.contains(Self.groupImuOrientation) {
            groups.append(Self.groupImuOrientation)
        }
        syncState = .adopted(groups)
    }

    // MARK: - Explicit push (user-initiated only)

    /// Write the whole active profile to the connected rocket.  This is the
    /// old #132 connect-time behaviour, kept as a deliberate user action:
    /// provisioning a fresh board, cloning a known-good setup onto a
    /// replacement computer, or making the unreported groups true after
    /// editing them offline.
    func pushProfileToRocket() {
        guard let device, let store, device.isConnected, !device.isBaseStation else {
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
                                  targetAlt: profile.pnTargetAltM,
                                  kpPos: profile.pnKpPos, kdVel: profile.pnKdVel,
                                  guidanceLaw: profile.pnGuidanceLaw)
        device.sendFinConfig(ringMode: profile.finRingMode,
                             servoAtSlot: profile.finServoAtSlot,
                             reverse: profile.finReverse,
                             rollReverse: profile.finRollReverse)
        device.sendCameraConfig(cameraType: profile.cameraType)
        device.sendImuOrientationConfig(profile.imuOrientSetting)
        device.sendImuRateConfig(profile.imuRateHz)
        device.sendSoundConfig(enabled: profile.soundsEnabled)
        device.sendPyroConfig(channels: [
            (profile.pyro1Enabled, profile.pyro1TriggerMode, profile.pyro1TriggerValue),
            (profile.pyro2Enabled, profile.pyro2TriggerMode, profile.pyro2TriggerValue),
            (profile.pyro3Enabled, profile.pyro3TriggerMode, profile.pyro3TriggerValue),
            (profile.pyro4Enabled, profile.pyro4TriggerMode, profile.pyro4TriggerValue),
        ])

        syncMagCal(profile: profile, device: device)
        syncSensorCal(profile: profile, device: device)

        // This profile now owns this board.
        store.update(profile.id) { $0.lastUsedUnitID = device.unitID }

        // Optimistic, like the rest of the app's config writes (no per-command
        // ack on this link).  Hold "syncing" briefly so the badge is visible.
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.8) { [weak self] in
            guard let self, case .syncing = self.syncState else { return }
            self.syncState = .synced
        }
    }

    // MARK: - The precedence rule

    // Group labels for the "what the rocket disagreed about" line.  Groups,
    // not fields: "PID gains" reads better on a phone than five numbers, and
    // the settings screen is organised the same way.
    static let groupServoTrim      = "Servo trim"
    static let groupServoTiming    = "Servo timing"
    static let groupPidGains       = "PID gains"
    static let groupServoControl   = "Servo control"
    static let groupGainSchedule   = "Gain schedule"
    static let groupRollControl    = "Roll control"
    static let groupGuidanceEnable = "Guidance"
    static let groupCamera         = "Camera"
    static let groupImuOrientation = "IMU orientation"
    static let groupImuRate        = "IMU rate"
    // #915 firmware follow-up: reachable only once the rocket reports them.
    static let groupServoTrim24    = "Servo trim 2-4"
    static let groupFinTravel      = "Fin travel"
    static let groupFinLayout      = "Fin layout"
    static let groupSounds         = "Sounds"
    static let groupGuidanceParams = "Guidance parameters"
    static let groupRollProfile    = "Roll profile"

    /// Reconcile a profile against what the rocket reported — rocket wins —
    /// and name the groups that differed.  Pure, so the precedence rule is
    /// testable without a peripheral (mirrors magCalSyncAction).
    ///
    /// Only fields the readback actually echoes are touched.  Anything the
    /// rocket doesn't report (see `unreportedGroups`) is left exactly as the
    /// profile has it, because the alternative — resetting it to a default we
    /// invented — would be a silent change of its own.
    ///
    /// Comparisons run at the precision the value crossed the wire at, so a
    /// profile Kp of 0.12 doesn't "differ" from a rocket reporting 0.1200
    /// every single connect.
    static func adopt(_ p: inout RocketProfile, from cfg: RocketConfig) -> [String] {
        var changed: [String] = []

        if p.servoBias1 != cfg.servoBias1 {
            p.servoBias1 = cfg.servoBias1
            changed.append(groupServoTrim)
        }

        if p.servoHz != cfg.servoHz || p.servoMinUs != cfg.servoMinUs
            || p.servoMaxUs != cfg.servoMaxUs {
            p.servoHz = cfg.servoHz
            p.servoMinUs = cfg.servoMinUs
            p.servoMaxUs = cfg.servoMaxUs
            changed.append(groupServoTiming)
        }

        if !same(p.pidKp, cfg.pidKp, decimals: 4)
            || !same(p.pidKi, cfg.pidKi, decimals: 4)
            || !same(p.pidKd, cfg.pidKd, decimals: 4)
            || !same(p.pidMinCmd, cfg.pidMinCmd, decimals: 1)
            || !same(p.pidMaxCmd, cfg.pidMaxCmd, decimals: 1) {
            p.pidKp = cfg.pidKp
            p.pidKi = cfg.pidKi
            p.pidKd = cfg.pidKd
            p.pidMinCmd = cfg.pidMinCmd
            p.pidMaxCmd = cfg.pidMaxCmd
            changed.append(groupPidGains)
        }

        if p.servoControlEnabled != cfg.servoEnabled {
            p.servoControlEnabled = cfg.servoEnabled
            changed.append(groupServoControl)
        }

        if p.gainScheduleEnabled != cfg.gainScheduleEnabled {
            p.gainScheduleEnabled = cfg.gainScheduleEnabled
            changed.append(groupGainSchedule)
        }

        // The three roll gains ride sentinels meaning "firmware default"
        // (#253).  When the rocket sends those, RocketConfig holds the app's
        // defaults rather than the vehicle's values — adopting them would
        // overwrite the profile with a number nobody chose.
        var rollDiffers = p.useAngleControl != cfg.useAngleControl
            || p.rollDelayMs != cfg.rollDelayMs
        if cfg.rollGainsReported {
            rollDiffers = rollDiffers
                || !same(p.rateCapDps, cfg.rateCapDps, decimals: 1)
                || !same(p.kpAngle, cfg.kpAngle, decimals: 2)
                || !same(p.integralSepThreshold, cfg.integralSepThreshold, decimals: 1)
        }
        if rollDiffers {
            p.useAngleControl = cfg.useAngleControl
            p.rollDelayMs = cfg.rollDelayMs
            if cfg.rollGainsReported {
                p.rateCapDps = cfg.rateCapDps
                p.kpAngle = cfg.kpAngle
                p.integralSepThreshold = cfg.integralSepThreshold
            }
            changed.append(groupRollControl)
        }

        // Enable flag only — the PN parameters behind it aren't reported.
        if p.guidanceEnabled != cfg.guidanceEnabled {
            p.guidanceEnabled = cfg.guidanceEnabled
            changed.append(groupGuidanceEnable)
        }

        if p.cameraType != cfg.cameraType {
            p.cameraType = cfg.cameraType
            changed.append(groupCamera)
        }

        // nil = this firmware doesn't report it; keep what the profile has.
        if let orient = cfg.imuOrientSetting, p.imuOrientSetting != orient {
            p.imuOrientSetting = orient
            changed.append(groupImuOrientation)
        }
        if let rate = cfg.imuRateHz, p.imuRateHz != rate {
            p.imuRateHz = rate
            changed.append(groupImuRate)
        }

        let pyro: [(Bool, UInt8, Float)] = [
            (cfg.pyro1Enabled, cfg.pyro1TriggerMode, cfg.pyro1TriggerValue),
            (cfg.pyro2Enabled, cfg.pyro2TriggerMode, cfg.pyro2TriggerValue),
            (cfg.pyro3Enabled, cfg.pyro3TriggerMode, cfg.pyro3TriggerValue),
            (cfg.pyro4Enabled, cfg.pyro4TriggerMode, cfg.pyro4TriggerValue),
        ]
        let profilePyro: [(Bool, UInt8, Float)] = [
            (p.pyro1Enabled, p.pyro1TriggerMode, p.pyro1TriggerValue),
            (p.pyro2Enabled, p.pyro2TriggerMode, p.pyro2TriggerValue),
            (p.pyro3Enabled, p.pyro3TriggerMode, p.pyro3TriggerValue),
            (p.pyro4Enabled, p.pyro4TriggerMode, p.pyro4TriggerValue),
        ]
        for i in 0..<4 where profilePyro[i].0 != pyro[i].0
            || profilePyro[i].1 != pyro[i].1
            || !same(profilePyro[i].2, pyro[i].2, decimals: 1) {
            changed.append("Pyro \(i + 1)")
        }
        p.pyro1Enabled = pyro[0].0; p.pyro1TriggerMode = pyro[0].1; p.pyro1TriggerValue = pyro[0].2
        p.pyro2Enabled = pyro[1].0; p.pyro2TriggerMode = pyro[1].1; p.pyro2TriggerValue = pyro[1].2
        p.pyro3Enabled = pyro[2].0; p.pyro3TriggerMode = pyro[2].1; p.pyro3TriggerValue = pyro[2].2
        p.pyro4Enabled = pyro[3].0; p.pyro4TriggerMode = pyro[3].1; p.pyro4TriggerValue = pyro[3].2

        // ── The #915 firmware follow-up groups ─────────────────────────────
        // Each is nil on a rocket that cannot report it (pre-report firmware,
        // or the mini, which has none of this hardware).  nil means "we don't
        // know", and the only honest thing to do with that is leave the
        // profile alone — resetting it to an invented default would be a
        // silent change dressed up as a reconciliation.

        if let e = cfg.servoExtras {
            if p.servoBias2 != e.bias2 || p.servoBias3 != e.bias3
                || p.servoBias4 != e.bias4 {
                p.servoBias2 = e.bias2
                p.servoBias3 = e.bias3
                p.servoBias4 = e.bias4
                changed.append(groupServoTrim24)
            }

            // #449 made the profile store ONE travel number, from which the
            // endpoints are derived symmetrically.  An asymmetric rocket-side
            // cal therefore collapses to its span — the profile has no way to
            // express it, and inventing one here would be a bigger change
            // than the readback is asking for.
            let travel = e.finMaxDeg - e.finMinDeg
            if !same(p.finTravelDeg, travel, decimals: 2) {
                p.finTravelDeg = travel
                changed.append(groupFinTravel)
            }

            // Azimuths come back as degrees; the profile stores the ring slot
            // each servo sits in, which is what the GUI edits.
            let slots = Self.slotsFromAzimuths(e.finAzimuths)
            let reverse = (0..<4).map { e.finReverseMask & (1 << $0) != 0 }
            let rollReverse = (0..<4).map { e.finRollReverseMask & (1 << $0) != 0 }
            let ringMode = Self.ringModeFromAzimuths(e.finAzimuths)
            if let slots, p.finServoAtSlot != slots || p.finReverse != reverse
                || p.finRollReverse != rollReverse || p.finRingMode != ringMode {
                p.finServoAtSlot = slots
                p.finReverse = reverse
                p.finRollReverse = rollReverse
                p.finRingMode = ringMode
                changed.append(groupFinLayout)
            }

            if p.soundsEnabled != e.soundsEnabled {
                p.soundsEnabled = e.soundsEnabled
                changed.append(groupSounds)
            }
        }

        if let g = cfg.guidanceExtras {
            if !same(p.pnNavGain, g.navGain, decimals: 2)
                || !same(p.pnMaxAccel, g.maxAccel, decimals: 1)
                || !same(p.pnAccelToFin, g.accelToFin, decimals: 2)
                || !same(p.pnMaxFinDeg, g.maxFinDeg, decimals: 1)
                || !same(p.pnMinSpeed, g.minSpeed, decimals: 1)
                || p.pnCoastDelayMs != g.coastDelayMs
                || p.pnTargetMode != g.targetMode
                || !same(p.pnTargetE, g.targetE, decimals: 1)
                || !same(p.pnTargetN, g.targetN, decimals: 1)
                || !same(p.pnTargetAltM, g.targetAltM, decimals: 1)
                || !same(p.pnKpPos, g.kpPos, decimals: 2)
                || !same(p.pnKdVel, g.kdVel, decimals: 2)
                || p.pnGuidanceLaw != g.guidanceLaw {
                p.pnNavGain = g.navGain
                p.pnMaxAccel = g.maxAccel
                p.pnAccelToFin = g.accelToFin
                p.pnMaxFinDeg = g.maxFinDeg
                p.pnMinSpeed = g.minSpeed
                p.pnCoastDelayMs = g.coastDelayMs
                p.pnTargetMode = g.targetMode
                p.pnTargetE = g.targetE
                p.pnTargetN = g.targetN
                p.pnTargetAltM = g.targetAltM
                p.pnKpPos = g.kpPos
                p.pnKdVel = g.kdVel
                p.pnGuidanceLaw = g.guidanceLaw
                changed.append(groupGuidanceParams)
            }
        }

        if let wps = cfg.rollWaypoints {
            let differs = wps.count != p.rollWaypoints.count
                || zip(wps, p.rollWaypoints).contains {
                    !same($0.timeSeconds, $1.timeSeconds, decimals: 2)
                        || !same($0.angleDeg, $1.angleDeg, decimals: 1)
                }
            if differs {
                p.rollWaypoints = wps.map {
                    RollWaypoint(timeSeconds: $0.timeSeconds, angleDeg: $0.angleDeg)
                }
                changed.append(groupRollProfile)
            }
        }

        return changed
    }

    /// Ring mode from the reported azimuths: "+" is the on-axis set
    /// {0,90,180,270}, "×" the 45°-rotated one.  Anything else is treated as
    /// "+" — the profile has no third mode, and the azimuths still round-trip
    /// through finServoAtSlot.
    static func ringModeFromAzimuths(_ az: [Float]) -> UInt8 {
        let rotated = az.contains { a in
            let m = (a.truncatingRemainder(dividingBy: 90) + 90)
                .truncatingRemainder(dividingBy: 90)
            return abs(m - 45) < 1
        }
        return rotated ? 1 : 0
    }

    /// Invert the app's slot→servo mapping from the azimuths the FC reports.
    /// `finServoAtSlot[s]` is the servo (1-4) sitting at ring slot s, and the
    /// FC is told each servo's azimuth — so slot index is the azimuth's
    /// position in the ring.  Returns nil if the azimuths don't describe four
    /// distinct slots, in which case the layout is left alone rather than
    /// guessed at.
    static func slotsFromAzimuths(_ az: [Float]) -> [Int]? {
        guard az.count == 4 else { return nil }
        var slots = [Int](repeating: 0, count: 4)
        var filled = Set<Int>()
        for (servoIdx, a) in az.enumerated() {
            let norm = (a.truncatingRemainder(dividingBy: 360) + 360)
                .truncatingRemainder(dividingBy: 360)
            // FLOOR, not round-to-nearest: a slot IS a quadrant of the ring,
            // and the "×" layout's azimuths (45/135/225/315) land exactly on
            // rounding ties — where Swift breaks away from zero and Kotlin
            // breaks to even, so the two platforms derived DIFFERENT fin
            // mappings from the same rocket.  Flooring has no tie to break
            // and gives 0...3 for both ring modes.
            let slot = min(max(Int((norm / 90).rounded(.down)), 0), 3)
            if filled.contains(slot) { return nil }   // two servos, one slot
            filled.insert(slot)
            slots[slot] = servoIdx + 1                // servos are 1-based
        }
        return filled.count == 4 ? slots : nil
    }

    /// Equal once both sides are rounded to the decimals the value is
    /// serialised at in the readback JSON.
    static func same(_ a: Float, _ b: Float, decimals: Int) -> Bool {
        let scale = powf(10, Float(decimals))
        return (a * scale).rounded() == (b * scale).rounded()
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
