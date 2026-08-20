# #848 — Drive P4_EN_HOLD in flight (+ the OC half of #825)

**Hazard (#825, verified):** the OC is the sole holder of the FC's power rail
(`PWR_PIN` → D9 → U30 EN). Any OC fault reset releases it; R84/C105 give
~0.8 s of grace; the OC reboots into `setup_oc()` which unconditionally drives
the rail LOW and waits for a BLE command that cannot come during ascent. The
FC — and all four pyro channels — power off mid-flight. V9 fitted the hardware
fix: FC GPIO5 (`P4_EN_HOLD`) is the second anode of the D9 diode-OR, so the FC
can hold its own rail. Firmware never drove it.

**Electrical facts (from #825's adversarial verification):** D9 is
common-cathode — the FC driving HIGH holds the rail regardless of the OC;
LOW and high-Z are electrically identical (an anode cannot pull the rail
down). So "release" is simply "stop holding"; there is no way for this pin to
power anything off.

## FC side (#848)

- **Assert on entering INFLIGHT** — the real launch transition and the
  reboot-recovery restore path both. Sim flights assert too: it exercises the
  mechanism on the bench with zero risk (release follows at sim landing, and
  the pin cannot cut power in any state).
- **Assert = drive HIGH + `gpio_hold_en()`.** The P4 supports pad hold
  (`SOC_GPIO_SUPPORT_FORCE_HOLD`), which latches the pad state THROUGH the
  FC's own panic/watchdog resets — an FC crash in flight no longer even
  starts the 0.8 s rail decay. Without hold, an FC reboot would race the
  R84/C105 window with bootloader time.
- **Release on entering LANDED**: `gpio_hold_dis()` + drive LOW. Post-landing
  rail security is the OC re-assert's job (below) — keeping hold after
  landing would make app power-off impossible without pulling the battery.
- **Boot reconciliation** (pwr_hold_policy.h): after state resolution —
  INFLIGHT → re-assert; latched hold + OC answering → release; latched hold +
  OC silent → **KeepHold** (downed-rocket tracker mode; battery pull to power
  off). KeepHold is re-reconciled at runtime: the moment `out_ready` latches,
  a non-INFLIGHT hold releases — so a transient miss of the single boot probe
  cannot leave the board un-power-off-able against a live OC. The same
  OC-liveness gate applies to the LANDED release.
- **V7/V8**: their board headers already carry `PWR_HOLD_PIN = -1` (added
  with the V9 header split); every call here is `>= 0`-guarded. No behaviour
  change on those boards.

## OC side (#825)

- **RTC-memory rail state** (`RTC_NOINIT_ATTR` + magic): updated at every
  rail toggle, plus a `deliberate_off` flag set immediately before the #9
  power-off `esp_restart()`.
- **Early re-assert, before the boot `delay(500)`**: if
  `esp_reset_reason() != ESP_RST_POWERON`, the magic validates, the rail was
  ON, and the restart was not the deliberate power-off → drive `PWR_PIN` HIGH
  first thing, set `pwr_pin_on`, and run the cmd-8 power-on init path once
  the stack is up. The fault-reset gap becomes bootloader-time only,
  comfortably inside R84/C105's ~0.8 s.
- Side effect (intended): an OC self-OTA restart no longer power-cycles a
  powered-on FC.

## Defense in depth

| Fault | Before | After |
|---|---|---|
| OC fault reset in flight | FC + pyro dead ~0.8 s later, ballistic | FC holds its own rail; OC also re-asserts on reboot — telemetry resumes |
| FC fault reset in flight | (rail stays, FC reboots cold) | pad hold keeps rail solid; boot reconciliation restores or releases |
| OC fault reset post-landing | FC dead, GNSS tracker lost | OC re-asserts; FC reboots; tracker resumes |
| Deliberate power-off | works | unchanged (hold released outside flight; deliberate_off flag) |

## Not doing

- No wire-protocol change: assert/release are boot-log lines; the app's
  existing power state view is unchanged. A status bit can ride a later
  protocol rev if bench work wants it.
- No hold outside INFLIGHT (ground tests/ARM included): with pyro armed on
  the pad, an OC reset dropping the rail is SAFE (FC off = nothing fires),
  and un-power-off-able ground states are an operator trap.

## Implementation notes (2026-08-20)

- Release sites are LANDED (one-shot block, reached by both the debounced and
  10-min-timeout paths), **`resetFlightStateForSim`** (a sim abort exits
  INFLIGHT without passing LANDED — found during implementation), and boot
  reconciliation. Assert sites: `enterInflight` + boot reconciliation
  (covers the recovery restore, which bypasses `enterInflight`).
- The boot reconciliation KeepHold row (orphaned hold, OC dead) is
  deliberate tracker mode: battery pull is the power-off. Logged at ERROR.

### Known quirks (accepted, not bugs)

1. **Power-off during a bench sim is deferred.** Cmd-8 OFF while a sim is
   INFLIGHT: the OC reboots rail-LOW (deliberate_off honored), but the FC's
   hold keeps the rail up until the sim lands or is reset — then the release
   drops it. Nothing can fire (sim), and the delay is bounded by the sim
   duration / 10-min timeout. Real flights have no power-off path anyway.
2. **Re-assert timing is bounded by measurement, not assumption.** The OC
   V9 image runs the PSRAM memtest before `app_main`, which spends part of
   the decay window. The window itself is longer than the header's "~0.8 s"
   shorthand: U30's EN threshold (~0.8 V typ) puts the R84/C105 decay from
   3.3 V at t = RC·ln(3.3/0.8) ≈ **1.4 s**. Still: measure reset→re-assert on
   the first article (#847), and drop `SPIRAM_MEMTEST` after first-article
   validation if the margin is thin. PRELAUNCH remains OC-restore-only by
   design (the FC hold is INFLIGHT-only; a pad rail loss is a scrub, not a
   hazard).
3. **Brownout retry is bounded** (`kMaxRestoreAttempts = 3`): a pack that
   sags under the restored load stands down to the stable rail-off idle
   after three consecutive failed restores instead of cycling forever.
4. **OC mid-flight reboot restores telemetry and pyro commands, not the
   active flight file.** The re-assert + `initPeripherals()` brings back the
   link, but the OC's in-RAM flight state is gone: post-reboot frames drop
   until the next flight is prepared. The pre-reboot portion is recovered by
   the existing brownout scanner (allocated-but-unindexed range) on the next
   boot. Continuing the SAME flight file across an OC reboot is #846-adjacent
   follow-up work, not part of this change.
