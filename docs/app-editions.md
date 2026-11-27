# App editions: store and dev

The iOS and Android apps build as two editions from the same `main`:

| | Store edition | Dev edition |
|---|---|---|
| Who it is for | Customers | Development, bench and flight testing |
| Boards | Tinker-Beetle only | Every board |
| Name on the phone | Tinker Rocket | Tinker Rocket Dev |
| Icon | The rocket-bug | The rocket-bug with a green **D** badge in the bottom-right corner |
| iOS bundle ID | `TinkerbugRobotics.TinkerRocket` | `TinkerbugRobotics.TinkerRocketApp` |
| Android application ID | `com.tinkerbug.rocket` | `com.tinkerbug.tinkerrocket` |
| iOS build | `Store` configuration, scheme **TinkerRocketApp Store** | `Debug` / `Release`, scheme **TinkerRocketApp** |
| Android build | `store` flavor (`:app:bundleStoreRelease`) | `dev` flavor (`:app:assembleDevDebug`, `:app:assembleDevRelease`) |
| Ships through | App Store; Google Play | Xcode / TestFlight internal testing; GitHub release APK |

The dev edition keeps the IDs the apps always had. Every phone that already has the
app updates in place and keeps its flights, rocket profiles and offline map tiles.
The store edition is a new app with its own IDs.

## The dev icon

The dev icon is the store icon with a white disc and a green D, drawn by
`tools/make_dev_icons.py` into `AppIconDev.appiconset` (iOS; the Debug and
Release configurations use it, Store keeps `AppIcon`) and `app/src/dev/res/`
(Android; the dev flavor's icons override main's, themed icon included).
Re-run the script whenever the store icon changes.

## Both on one phone

The two editions have different IDs, so they install side by side, each under its
own name. They do not share anything: each has its own flights, rocket profiles,
offline maps and list of known boards. Use one at a time near the boards. Both
reconnect to boards they already know, and both would announce the flight.

## What differs

The code sees one switch: `TR_STORE` (Swift compilation condition on iOS,
`BuildConfig.TR_STORE` on Android). Every rule that reads it lives in one file per
app, **AppEdition.swift** and **AppEdition.kt**, which are kept in step case for case.

A feature follows what the board has, not just the edition. The board files say it:
`board_m2.h` (Beetle V2) has no servo outputs and no camera port, while `board_v10.h`
(Mantis) has three servo outputs and a GoPro/RunCam port.

| Feature | Store edition | Dev edition |
|---|---|---|
| Servo control, PID gains, servo calibration, control mode, roll control, roll profile, guidance, fin layout | Hidden | Shown, except while a Beetle is connected |
| Servo Test, Ground Test, Drift Cast's "Send to Unit" (the guidance aim point; iOS only so far) | Hidden | Shown, except while a Beetle is connected |
| Camera settings, the Start Camera control, the Camera status badge, camera in the rocket summary, pre-flight "camera recording" check | Hidden | Shown, except while a Beetle is connected |
| Firmware catalog: flight and out computer images | Beetle images, plus images built for every board | All |
| Firmware catalog: base station images | All | All |
| A connected rocket that is not a Beetle | Connects; a notice says the board is not supported, and its board-specific screens stay hidden | Fully supported |
| Everything else: telemetry, maps, Drift Cast, flight logs, pyro, recovery, calibration, radio, simulation, Virtual Rocket | Same | Same |

The app works out which board it is talking to from, in order: the flight computer's
provisioned revision (`fc_board`), the out computer's (`oc_board`), then the board
suffix of the running firmware (`-m2`, `-v10`). The letter decides: **M** is the
Beetle (M2 = V2, M3 = V3) and **V** is the Mantis and the V boards before it.
Nothing connected, a base-station relay, or a board that names no family counts as
unknown. The dev edition shows everything for an unknown board. The store edition
shows the Beetle set and no warning, because a Beetle reads as unknown until its
identity arrives.

## Building

iOS, from `TinkerRocketApp/`:

```bash
xcodebuild archive -project TinkerRocketApp.xcodeproj -scheme "TinkerRocketApp Store" -archivePath build/TinkerRocket-Store.xcarchive -allowProvisioningUpdates
```

In Xcode, pick the **TinkerRocketApp Store** scheme and use Product › Archive. Running
that scheme installs the store edition on a phone or simulator. The
**TinkerRocketApp** scheme runs, tests and archives the dev edition, as before.

Android, from `TinkerRocketAndroid/`:

```bash
./gradlew :app:bundleStoreRelease
```

`./gradlew test` runs the whole unit suite once per variant, store variants included.
`TR_ANDROID_APP_ID_SUFFIX` still adds a suffix to debug builds of either flavor.

CI: iOS tests run the dev edition, and a second step builds the store edition. Android
tests run every variant. The Android release workflow builds and publishes the dev
edition APK, as before. The store edition's Play upload is not wired into CI yet
(see `docs/android-release-signing.md`).

## Before the first store upload

- **App Store Connect:** create the app record for `TinkerbugRobotics.TinkerRocket`.
  Automatic signing registers the bundle ID on the first archive.
- **Google Play:** create the app for `com.tinkerbug.rocket`. It is a new package that no
  phone has sideloaded, so Play may generate its signing key.
- **App Review needs to see the app work.** Reviewers have no Beetle. The Virtual Rocket
  runs a full flight with no hardware, so point to it in the review notes and attach a
  short video of the app running a real Beetle.

## Adding a board-specific feature

1. Add a property to `AppFeatures` in both AppEdition files, with its rule for each
   edition.
2. Gate the screen on it (`device.features` on iOS, `rememberFeatures(session)` on
   Android).
3. Add the case to `AppEditionTests.swift` and `AppEditionTest.kt`.

When the Mantis is released, the store rules change in those two files: the store
edition then follows the board, as the dev edition does.
