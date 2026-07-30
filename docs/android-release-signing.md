# Android release signing — key custody and setup

*Android port plan §2.5. Source is public and forkable; the signing key is not. The key IS
the app's update identity: Android refuses any update whose signature differs from the
installed app, and a fork signed with its own key can build and run TinkerRocket but can
never impersonate an update to a real install. `applicationId com.tinkerbug.tinkerrocket`
is the other half of that identity.*

## Custody model

- The keystore lives **outside the repo** (suggested: `~/.tinkerrocket/release.keystore`)
  and reaches builds only via env vars. `.gitignore` blocks `*.keystore` / `*.jks` as
  belt-and-suspenders.
- **Two backups you control**, because until Play enrollment there is no escrow and a lost
  keystore means every phone must uninstall (losing profiles, saved flights, offline tiles)
  to accept a new key:
  1. a password-manager entry holding the base64'd keystore file + both passwords;
  2. one offline copy (USB stick / second machine).
  GitHub secrets are *not* a backup — they can't be read back out, and losing GitHub access
  is one of the failure modes the backup exists for.
- CI (`android-release.yml`) gets the keystore as a base64 repo secret, decodes it to a
  runner-temp file, signs on `android-v*` tag push, and refuses to ship a debug-signed APK.

## One-time setup (operator runs these; keeps secrets out of transcripts/history)

Mint the key (prompts interactively for the two passwords — use the password manager's
generator; 25+ years validity is deliberate, the key must outlive the fleet):

```bash
mkdir -p ~/.tinkerrocket && keytool -genkeypair -v -keystore ~/.tinkerrocket/release.keystore -alias tinkerrocket -keyalg RSA -keysize 4096 -validity 10000 -dname "CN=TinkerRocket, O=Tinkerbug Robotics"
```

Add the four repo secrets (each prompts on stdin; paste from the password manager):

```bash
base64 -i ~/.tinkerrocket/release.keystore | gh secret set TR_ANDROID_KEYSTORE_B64
```

```bash
gh secret set TR_ANDROID_KEYSTORE_PASSWORD
```

```bash
gh secret set TR_ANDROID_KEY_ALIAS --body tinkerrocket
```

```bash
gh secret set TR_ANDROID_KEY_PASSWORD
```

Then back up: password-manager entry with the base64 blob + both passwords, plus one
offline copy of the file.

## Cutting a release

```bash
git tag android-v1.0.0 && git push origin android-v1.0.0
```

CI runs the JVM suite, builds `:app:assembleRelease` signed with the release key, verifies
the signer isn't the debug cert, and attaches `TinkerRocket-android-v1.0.0.apk` to a GitHub
release. Bump `versionCode`/`versionName` in `app/build.gradle.kts` before tagging —
`versionCode` must increase monotonically (Play requires it; sideloads want it too so
"update" installs work without uninstalling).

Local signed build, when needed:

```bash
export TR_ANDROID_KEYSTORE=~/.tinkerrocket/release.keystore TR_ANDROID_KEY_ALIAS=tinkerrocket
```

then set the two password vars from the password manager and run
`./gradlew :app:assembleRelease` in `TinkerRocketAndroid/`.

## Forks

Nothing to configure: with no `TR_ANDROID_KEYSTORE` in the environment, `assembleRelease`
falls back to the debug signing config — installable everywhere, distinct identity, cannot
update a real install. Forks that want to distribute must mint their own key and rename
`applicationId`.

## The Play Store path (when the app matures)

Play App Signing is mandatory for new apps: Google holds the **app signing key** in escrow
and you sign uploads with an **upload key** (recoverable through support if lost). The
wrinkle for our sideload-first fleet: if Google *generates* the app signing key, the Play
build's signature won't match sideloaded installs — every fleet phone would need an
uninstall (data loss) to switch to Play.

**The plan: enroll with "use existing key"** — upload THIS release key as the Play app
signing key. Sideloads and Play installs then share a signature, fleet phones migrate to
Play seamlessly, and Google's escrow takes over long-term custody from that day. Until
enrollment, the password-manager + offline backups above are the only copies in the world.

Also required for Play, handled at that time: AAB uploads instead of APKs (a one-job
addition to the workflow), and the closed-testing gauntlet for new personal accounts
(20 testers × 14 days) — the reason v1.0 ships via GitHub Releases (plan §1 Distribution).

## Debug builds and `run-as`

Bench phones run debug builds signed by each dev machine's auto-generated
`~/.android/debug.keystore`. Debug builds are `debuggable`, which is what allows `adb
run-as` file pulls during bench work (used by the Checkpoint A CSV-parity test) — release
builds correctly forbid that. The shadow-outing phone should carry the **release-signed**
APK: it's the artifact v1.0 actually ships, and the outing is its validation pass.
