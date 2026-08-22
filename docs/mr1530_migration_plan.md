# MR-15-30 Migration Plan — Replacing CONEX-AG-M100D

Status: updated 2026-08-18 (v2). Hardware not yet ordered. This plan is
written from the existing codebase (`motion/motion_controller.py`,
`motion/real_conexagap_motion.py`, `scan/calibrate_scan_area.py`,
`scan/scan_manager.py`, `config.yaml`) plus what Optotune confirmed in the
Aug 14–18 quote thread, plus (v2) the publicly published **MR-E-3
Development Kit Operation Manual** found on optotune.com — this resolved
most of what section 3 originally called "blocking on vendor docs." One
real fork remains: reading back ACTUAL (measured) mirror position needs
either a separate Firmware Documentation zip this session couldn't
download (network-restricted), or Optotune's official Python SDK — see
section 3b, ARCHITECTURE DECISION NEEDED.

## 1. What stays untouched

`scan_manager.py` calls only the abstract `MotionController` interface —
`home()`, `resume()`, `move_to()`, `wait_for_settle()`, `get_position()`,
`check_limits()`, `close()`. No CONEX-specific code lives there. Confirmed by
grep: zero references to CONEX/conex in `scan_manager.py`. **This file needs
no changes for the swap**, same as it needed none for the 8742→CONEX-AGAP
cutover.

`calibrate_scan_area.py`'s core algorithm — jog to each wafer edge in the
controller's own native rotational unit via `calibration_jog_deg()` /
`get_position_deg()`, then derive `deg_per_mm_x/y` from the measured degree
separation and a known real-world distance — is written against the abstract
interface, not CONEX internals. If the new backend implements
`calibration_jog_deg()` and `get_position_deg()` following the same contract
`MotionController` defines, this script's algorithm does not need to be
rewritten.

## 2. What must change

**New backend module**: `motion/real_mr1530_motion.py`, implementing
`MotionController` — `home()`, `resume()`, `move_to()`, `get_position()`,
`wait_for_settle()`, `zero_here()`, plus the calibration-degree extensions
(`calibration_jog_deg()`, `get_position_deg()`, `get_absolute_position_deg()`)
that `calibrate_scan_area.py` depends on. Use `real_conexagap_motion.py` as
the structural template — same shape: config parsing → serial connect →
ID-confirm → limits-sanity-check on construction, then the same
motion_enabled fail-closed interlock, the same "validate both axis targets
before sending either" pattern, the same `MotionFault` vs `AxisStateUnknown`
distinction on a stuck move.

**Factory wiring** in `motion/motion_controller.py`: add a `controller_type
== "mr1530"` (or similar) branch to `get_motion_controller()`, alongside
(not replacing) the `conex_agap` branch — same pattern used when
`conex_agap` was added next to `newport_8742`.

**Config**: new `motion:` block in `config.yaml` / `config.example.yaml`.
Proposed key reuse vs. new keys:

- `controller: mr1530` (or `optotune_mr1530`)
- `port` — new physical COM port, separate from the CONEX one; keep the old
  `conex_agap` block commented for rollback, exactly as `newport_8742` was
  kept commented after the CONEX-AGAP cutover.
- `deg_per_mm_x` / `deg_per_mm_y` — same key names, same semantic
  (degrees of mirror tilt per mm of spot travel). **The values themselves do
  not carry over** — they're a function of mirror-to-target throw distance,
  not the specific motor, and the CONEX's `0.05`-ish placeholder was already
  calibrated for a specific mount geometry. Recalibrate on-site via
  `calibrate_scan_area.py`'s degree-first flow either way.
- `axis_x` / `axis_y` — **RESOLVED (v2): not needed.** The MR-E-3 addresses
  X/Y directly (`X=`/`Y=`/`XY=` commands), unlike the CONEX's U/V
  letter-to-axis mapping. `invert_x`/`invert_y` is the only wiring-direction
  knob this driver needs.
- `invert_x` / `invert_y` — carry the pattern over, re-verify on real
  hardware, don't assume the CONEX's `invert_y: true` applies to different
  wiring.
- `soft_limits` (mm) — currently unrescaled 8742-era placeholder
  (±1000 mm) even for the CONEX; needs a real value for the MR-15-30 based
  on the actual mirror-to-target geometry. **Resolved (v2), partially**:
  there's no live hardware-limit *query* the way `ConexAGAPController`
  checks live SL/SR — instead the firmware self-clamps out-of-range targets
  (`OL`/`OU` response codes) and the reachable range is the fixed unit
  circle X²+Y²≤1 (not two independent per-axis limits). v2's
  `_validate_normalized_target()` checks against that fixed circle before
  sending; `motion.soft_limits` in mm still needs a real value so the
  scan-grid math itself stays inside a sane physical area, but it's a
  simpler check than the CONEX's live-queried limits were.
- Drop `controller_address` (CONEX-AGAP-specific, ties to RS-485
  addressing) unless the MR-E-3 protocol has an equivalent concept —
  unconfirmed.
- `homing_required` — already a dead config key for `conex_agap` (flagged in
  the 2026-08-05 audit); don't carry the same mistake forward without
  actually wiring it up or deciding to drop it.

**`calibrate_scan_area.py` numeric constants** — `JOG_STEP_DEFAULT_DEG` (0.05),
`JOG_STEP_MAX_DEG` (0.3), `CLEARANCE_CHECK_STEP_DEG` (0.1), and
`JOG_CHECKPOINT_INTERVAL_MAX_DEG` (0.76) are all explicitly sized to the
CONEX's ~0.76–1.5° single-axis travel (the comments say so directly — e.g.
"matches the controller's own single-direction travel"). None of these are
*unsafe* for a ±25° device — they're conservative, so at worst calibration on
the MR-15-30 is slower than it needs to be, jogging in much smaller steps
than the device's real range allows. Worth revisiting once the real
mirror-to-target throw distance is known, so the jog step sizes are sized to
the actual optical field of view being calibrated, not left at CONEX-era
defaults by accident. Also update the "8742 note" / CONEX-specific docstring
language once the new controller is real, so a future reader isn't misled
about which controller the script currently assumes.

## 3. Protocol — now confirmed (v2, from Optotune's published Operation Manual)

Found via web search, not the email thread: Optotune publishes an **"MR-E-3
Development Kit Operation Manual"** (Rev 1.0, 2025-12-01) on
optotune.com/product/mr-e-3/, plus a firmware download bundle at
optotune.com/software-download/ containing a separate **"MR-E-3 Firmware
Documentation"** zip and an official **"MR-E Python SDK"** zip (pure Python
per its changelog, not a .NET wrapper). This session's sandbox couldn't
download the zips directly (network access is allowlisted to package
registries, not arbitrary vendor domains) but WebFetch could read the
Operation Manual PDF itself. Confirmed from it:

- **Serial framing**: 256000 baud, 8 data bits, 1 stop bit, no parity,
  `\r\n` terminator, 64-byte max message size, commands case-insensitive.
- **Handshake**: `START` → `OK`.
- **Move commands** (Simple Serial mode, Table 3): `X=%f`, `Y=%f`,
  `XY=%f;%f` — NOT plain degrees. Values are normalized to a **unit
  circle** (X²+Y²≤1): "+50° optical deflection corresponds to a value of
  +1," i.e. `x = tan(optical_deg) / tan(50°)`, and optical deflection is
  2× mechanical mirror tilt (standard fold-mirror doubling), so the
  quoted ±25° mechanical spec is exactly the ±1 normalized range. The
  firmware auto-clamps an out-of-circle target to the nearest edge point
  rather than rejecting it — `real_mr1530_motion.py` v2 validates and
  rejects before sending instead, consistent with this project's existing
  fail-loud philosophy.
- **Response codes** (Table 4): `OK` / `NO` / `OL` (lower limit clamp) /
  `OU` (upper limit clamp) / `ERROR` — every Simple Serial command gets an
  explicit reply, unlike the CONEX's fire-and-forget `PA`.
- **Settled status**: `STATUS` returns a 32-bit hex register (Table 5);
  **bit 4 is "Mirror not stable."** This replaces the CONEX's TS-state-code
  polling. (The 0x1007 "register" mentioned in the email thread turned out
  to be Pro-mode/SPI register addressing, a different thing from this
  Simple-Serial STATUS bit — see 3b.)
- **Homing**: `XY=0;0` drives to the unit circle's mechanical/optical
  center — always valid, not unit-specific the way the CONEX's
  factory-calibrated electrical zero was. This is what `hard_home: true`
  now does in v2.

All of this is implemented in `real_mr1530_motion.py` v2: `home()`
(hard_home path only), `move_to()`, and `wait_for_settle()` are real, not
TODO stubs, against this confirmed protocol.

## 3b. Architecture decision needed — reading ACTUAL position

The Operation Manual's Simple Serial command table has **no command that
reads back measured position** — only `X=`/`Y=`/`XY=` to *set* commanded
position. Reading actual closed-loop position requires "Pro mode"
(`GOPRO`/`GOPROCRC`), a binary register-addressed protocol the Operation
Manual explicitly defers to the separate Firmware Documentation zip: *"For
full description of the MR-E-3 register map, please refer to MR-E-3
Firmware documentation."* That's the same document referenced in the SPI
section, which does list `0x2300`/`0x2301` as "Optical feedback read
registers X, Y" over SPI — plausibly the same registers apply over
Pro-mode serial, but the exact binary frame format for serial Pro mode
isn't in the Operation Manual, and this session couldn't fetch the zip to
check.

Two ways to close this, genuinely different trade-offs, worth deciding
deliberately rather than defaulting into one:

1. **Get the Firmware Documentation zip** (link below) and hand-roll the
   Pro-mode binary protocol in pure pyserial — consistent with this
   project's established "no vendor SDK" precedent from the CONEX-AGAP
   integration. More work, but zero new dependency and full control over
   the exact bytes on the wire (useful given `_wait_move()`'s timeout path
   currently has no documented stop command to fall back on — Pro mode
   might expose one).
2. **Use Optotune's official "MR-E Python SDK"** — pure Python (unlike the
   CONEX-AGAP's rejected `ConexAGAPCmdLib.dll`, a Windows-only .NET wrapper
   that was the actual reason "no vendor SDK" became this project's default
   — see `real_conexagap_motion.py`'s module docstring). Almost certainly
   already implements Pro-mode register reads correctly, and may also
   expose a real stop/abort command this driver currently lacks. Real
   dependency addition, and a deliberate departure from precedent — but
   that precedent was a reaction to a specific bad option, not a blanket
   rule.

**Direct download links** (couldn't fetch these directly from this
session's sandbox — pull them yourself and send back, or make the call on
option 1 vs. 2 above):
- Firmware Documentation zip: `https://files.optotune.com/hubfs/2025%20Website%20Downloads/Download%20Hub/MR-E-3_Firmware_Documentation_1.6.742180.zip`
- MR-E Python SDK zip: `https://145326430.fs1.hubspotusercontent-eu1.net/hubfs/145326430/2025%20Website%20Downloads/Download%20Hub/MR-E_PythonSDK_1.3.5434.zip`
- Operation Manual (already read into this plan): `https://145326430.fs1.hubspotusercontent-eu1.net/hubfs/145326430/2025%20Website%20Downloads/Download%20Hub/Optotune+MR-E-3+Development+Kit+Operation+manual.pdf`

Until this is decided, `get_position()`, `get_position_deg()`,
`get_absolute_position_deg()`, `zero_here()`, `calibration_jog_deg()`, and
the `hard_home: false` fiducial path of `home()` all raise
`NotImplementedError` in `real_mr1530_motion.py` v2 — deliberately, not
stubbed with last-commanded position as a stand-in. Given this device's
accuracy (0.15°) is ~65× worse than its repeatability (40 μrad), silently
substituting commanded for measured position would defeat the entire
reason this project calibrates against real closed-loop feedback.

## 4. What's done now vs. what's still open

Done in `real_mr1530_motion.py` v2: config parsing (including a hard
refusal to start without `deg_per_mm_x/y` explicitly set — no safe
placeholder exists for this device the way the CONEX had one), serial
connect + `START`/`GETID` handshake, `move_to()` (mm → mechanical deg →
optical deg → normalized XY, unit-circle validation before send),
`wait_for_settle()` (STATUS bit 4 polling, plus fails loud on current-limit
or thermal-limit bits instead of ignoring them), and `home()`'s
`hard_home: true` path. `motion_controller.py`'s factory has the `mr1530`
branch wired in. `config.example.yaml` has a draft commented block.

Still open: the position-readback decision (3b) — everything downstream of
it (fiducial homing, calibration jogging, `get_position()` for
`calibrate_scan_area.py` and `scan_manager.py`'s point logging) is blocked
until that's resolved. Also open: whether Pro mode exposes an explicit stop
command (`wait_for_settle()`'s timeout path currently has none to fall back
on, unlike the CONEX's `ST`) — worth checking once the Firmware
Documentation or SDK source is available. And: `calibrate_scan_area.py`'s
CONEX-tuned numeric constants (unchanged from v1 of this plan, see below).

## 5. Rollback / testing posture

Keep `real_conexagap_motion.py` and the `conex_agap` config path exactly as
they are — commented alongside, not deleted — same posture as the 8742 code
after its cutover. The CONEX-AGAP integration was verified pre-hardware
against a hand-written fake-serial simulator implementing the documented
ASCII protocol; the same approach applies here once the real protocol is
known, and is the only way to exercise this driver before physical hardware
arrives (lead time still unconfirmed as of 2026-08-18, see
`optotune_mr1530_fsm_evaluation` memory).
