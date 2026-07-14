# #526 — L2CAP CoC file transfer: plan

Produced by a 3-design / 4-adversarial-lens workflow (16 agents), then synthesised.
Every file:line below was verified against the real tree + `~/esp/esp-idf-v6.0` NimBLE source.

## Recommendation

Keep the download loop exactly where it is — blocking `while(!eof)` on **oc_loop**, inside
`beginPhoneIO()/endPhoneIO()`, with the #383 INFLIGHT gate still in the loop — and add L2CAP CoC
as a **second, explicitly-requested transport on the OC only**, selected by a new command id,
never inferred from channel state. GATT stays as the fallback.

## The four bugs the adversarial pass killed (all source-verified)

1. **chan* use-after-free.** `ble_l2cap_coc_send()` dereferences the channel before locking
   (`ble_l2cap_coc.c:740-750`) and NimBLE frees the chan on disconnect (`ble_l2cap.c:108-127`).
   A mutex around a cached `chan*` is an AB-BA deadlock against `ble_hs_lock`.
   → Fix: **confine every `ble_l2cap_*` call to the NimBLE host task** via a posted event. No mutex.

2. **`rc == 0` does not mean "on the wire", and msys exhaustion is destruction not backpressure.**
   `ble_l2cap_tx()` queues into `conn->bhc_tx_q` and returns 0 when the controller is full
   (`ble_l2cap.c:492-497`). msys exhaustion inside `continue_tx` takes `failed:`
   (`ble_l2cap_coc.c:626-628`) which **frees an SDU whose K-frames are already partly on air** →
   silent file corruption. The GATT path hits the same condition harmlessly because the caller
   keeps owning the flat buffer and just waits.
   → Fix: **`os_msys_num_free() >= watermark` gate on oc_loop before every submit.** Mandatory.

3. **Missing `BLE_L2CAP_EVENT_COC_ACCEPT`.** Server channel starts `sdu_rx = NULL`
   (`ble_l2cap_coc.c:400/:448`) but RX credits are granted anyway (`:427`) → the first byte the
   phone writes NULL-derefs on the host task in release builds.
   → Fix: handle ACCEPT + `ble_l2cap_recv_ready()`.

4. **Transport inferred from `chan_ != nullptr` is racy + un-cancellable.** CoreBluetooth has no
   `cancelL2CAPChannel`; a late `didOpen` desyncs app vs firmware and the download hangs at zero
   bytes with no timer armed.
   → Fix: **explicit command id.** (Both runner-up designs picked colliding ids 26/27/28 —
   26/27 are OC roll-profile at `main.cpp:6394/:6411`. Use **43/44**, free in the OC dispatch.)

## Architecture

- **New OC commands 43 (L2CAP_PSM_QUERY), 44 (FILE_DOWNLOAD_L2CAP).** OC command space is
  independent of the BS space.
- **PSM handshake, pull-only:** cmd 43 → OC loop replies `0xCE + psm(u16 LE) + ver` on the existing
  first-byte-multiplexed `file_ops` readback (no new GATT characteristic). App queries at connect
  (when `!isBaseStation`), caches, opens the channel lazily on first download, sends cmd 44 only
  **after** `didOpen`. PSM **0x0083** (LE dynamic range; IDF's 0x1002 is out of range).
- **Direction (verified):** phone = central → `CBPeripheral.openL2CAPChannel(psm)`; ESP32 =
  peripheral → `ble_l2cap_create_server()`. Server created **once** from `on_ble_hs_sync()`.
- **Send engine:** oc_loop reads NAND (unchanged), gates on msys watermark, builds one SDU into a
  **dedicated mempool** (size `min(1024, peer_coc_mtu)`), posts a `ble_npl_event`, blocks on a task
  notification. Host task does one O(1) `ble_l2cap_send()` + the DONE/STALLED/ERROR/DEAD state
  machine. No blocking NAND, no logging above WARN on the host task.
- **Framing (in-band, because iOS gives a byte stream with no SDU boundaries):**
  `[type u8][len u32 LE][payload]` — BEGIN{ver,status,size_hint,name} / DATA{packed AA55AA55 frame
  bytes, **reusing today's packer verbatim**} / END{bytes u32, crc32 u32, status u8}. Completion =
  END with status 0 and matching CRC32. Nothing else completes a download.
- **Stall budget: 30 s of no progress** (not `effectiveEventMs()*8` — CoC credits only return when
  the iOS app drains its InputStream, so a 500 ms budget kills healthy transfers on any UI hitch).

## base_station safety (why #380 cannot regress)

1. No blocking L2CAP send loop enters any path bs_loop runs — the producer stays on oc_loop.
2. The feature is not in the BS binary. **Trap the other designs missed:** an *absent*
   `CONFIG_BT_NIMBLE_L2CAP_COC_MAX_NUM` means `MYNEWT_VAL` = **2** (`esp_nimble_cfg.h:887`), i.e.
   CoC compiled *in*. BS gets an explicit `=0`, asserted present-and-0 in its generated sdkconfig.
3. Shared-component edits are additive + inert on BS: `sendFileChunk(..., bool abort=false)` keeps
   its signature; new methods declared **unconditionally** in the `.h` (guarding the header with
   `#if MYNEWT_VAL` is a hard preprocessor error, and `#ifdef CONFIG_...` is *true* on BS).
4. Host tests can't catch a BS transport regression (nothing in tests_cpp links TR_BLE_To_APP), so
   every shared-component step ends with a BS build + a CSV smoke test with telemetry flowing.

## Steps (each ends at a compile-verified / host-tested / bench-testable seam)

- **Step 0** — wire codes 43/44 + pure policy modules (`Crc32.h`, `FileStreamFraming.h`,
  `oc_l2cap_send_policy.h`) + gtests. No behaviour change. `check_ble_command_ids.py` stays green.
- **Step 1** — sdkconfig only: OC `COC_MAX_NUM=1`, `MSYS_1_BLOCK_COUNT=48`; BS explicit `=0`.
  Boot line prints `MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)`. Measures the RAM delta in isolation (~11 KB).
- **Step 2** — **ABORT flag** (chunk-header flags bit1): closes the #524-class truncation hole on
  the LEGACY path. Standalone value, ships even if CoC never does. (See open decision 2.)
- **Step 3** — CoC lifecycle + PSM handshake, no data path. **The big de-risk:** does iOS actually
  open a channel at 0x0083 on an unencrypted link, and what is `peer_coc_mtu`?
- **Step 4** — FW send engine + cmd-44 producer, behind a debug trigger.
- **Step 5** — iOS consumer (dedicated Thread + RunLoop, stream to `.part`, verify CRC, then rename)
  + fallback. **The oracle:** same flight over cmd 4 and cmd 44 → sha256 identical.
- **Step 6** — harden (yank phone, screen lock, INFLIGHT) + tune SDU/watermark on real numbers.

## Bench oracle

Download the same flight over cmd 4 and cmd 44 back-to-back, same phone, same distance:
**sha256 must be identical** (both use the same frame packer), then compare kB/s. Also A/B a
crash-**recovered** flight (page-rounded `final_bytes` + 0xFF pad) — the case a raw-stream design
would have silently diverged on. `msys_free_min` near zero is the corruption precursor and must be
visible before any throughput claim.
