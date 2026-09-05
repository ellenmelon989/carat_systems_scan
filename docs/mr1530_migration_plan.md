# MR-15-30 Migration Plan — Replacing CONEX-AG-M100D

Status: updated 2026-08-23 (v3). Hardware not yet ordered. This plan is
written from the existing codebase (`motion/motion_controller.py`,
`motion/real_conexagap_motion.py`, `scan/calibrate_scan_area.py`,
`scan/scan_manager.py`, `config.yaml`) plus what Optotune confirmed in the
Aug 14–18 quote thread, the publicly published **MR-E-3 Development Kit
Operation Manual** (v2), and now (v3) the user's downloaded **MR-E Python
SDK** (`optomdc`/`optokummenberg` wheels) and firmware bundle. Section 3b's
architecture fork is now **RESOLVED**: read the SDK's pure-Python source as
a correctness reference (not a runtime dependency), hand-port the exact
Pro-mode protocol into this driver. `real_mr1530_motion.py` v3 is now
functionally complete — every `MotionController` method implemented —
though still untested against real hardware. See section 3b and section 4.

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

## 3b. Position readback — RESOLVED (v3)

The Operation Manual's Simple Serial command table has **no command that
reads back measured position** — only `X=`/`Y=`/`XY=` to *set* commanded
position. Reading actual closed-loop position requires "Pro mode"
(`GOPRO`/`GOPROCRC`), a binary register-addressed protocol the Operation
Manual explicitly deferred to a separate Firmware Documentation zip.

The user supplied two downloaded folders directly: the firmware bundle
(`MR-E-3_Firmware_1.6.742180/`) and the **official MR-E Python SDK**
(`MR-E_PythonSDK_1.3.5434/`, two wheels: `optomdc` + `optokummenberg`).

- **Firmware folder was a dead end** — it contains only the compiled
  `.hex` firmware binary and a release-notes text file. Not the separate
  "Firmware Documentation" PDF/zip referenced in the manual; not useful for
  protocol reverse-engineering.
- **SDK folder had the actual answer.** Unzipped both `.whl` files (plain
  ZIP archives) and confirmed they're pure Python — no compiled
  extensions, same situation as `real_newport_motion.py` avoiding
  pylablib and `real_conexagap_motion.py` avoiding the CONEX-AGAP's
  Windows-only `.NET` DLL. That's a materially different case from those
  precedents, which existed specifically because those two vendor SDKs
  weren't pure Python. **Decision: read the SDK source as a correctness
  reference, do not add it as a runtime dependency.** Hand-port the exact
  protocol logic into this driver's existing pyserial-only pattern. This
  gets vendor-verified correctness (not a blind guess at the binary
  format) while keeping the project's "no vendor SDK dependency"
  convention intact.

**What was found, reading the SDK source directly:**

- Pro-mode frame format (`optoKummenberg/tools/parsing_tools.py`):
  `0x7E` + slave_addr(`0x00`) + command_id(1B) + size(1B) + payload +
  CRC(2B) + `0x7E`. Byte-stuffing: `0x7D`/`0x7E` inside the payload is
  escaped as `0x7D` followed by `byte XOR 0x20`. CRC is never actually
  computed by the firmware/SDK in practice (`CRC_ENABLED` is always
  False) — always `0x00 0x00`.
- Command IDs (`optoKummenberg/tools/definitions.py`): `GET_VALUE=0x11`,
  `SET_VALUE=0x10`, `SET_COMM_MODE=0x06`, plus others not needed here.
- **The actual position registers**: `optomdc/registers/mre3_registers.py`
  — the `RadialBasisFunction` system (sys_id `0x3B`) exposes **`0x3B00`
  ("Mirror coordinate X") and `0x3B01` ("Mirror coordinate Y")** as
  read-only floats, explicitly documented as the "mirror unary circle X/Y
  coordinate" — i.e. actual measured position, same normalized unit-circle
  coordinate system as the Simple Serial `X=`/`Y=`/`XY=` move commands.
  This is the single register pair the driver now reads.
  (An alternative, more roundabout path also exists via
  `SignalFlowManager.GetStageOutput()` reading the feedback signal-flow
  block — confirmed working in the SDK's own
  `MR-E-3_ReadBackSignalFlowValues.py` example — but 0x3B00/0x3B01 is
  simpler and was used instead.)
- **Correction to an earlier assumption**: the original email thread's
  mention of "register 0x1007" is `MRE3Status`'s generic board-fault
  register (channel faults, overheat, device-not-detected, over-current)
  — **not** a mirror-settled bit. It's unrelated to position or settling.
  The settle mechanism used by `wait_for_settle()` remains the Simple
  Serial `STATUS` command's bit 4, confirmed from the Operation Manual —
  that part of the v2 driver was already correct and is unchanged.
- **Mode switching is bidirectional and cheap**: ASCII `GOPRO` enters Pro
  mode; a Pro-mode `SET_COMM_MODE(0)` frame plus a fresh `START`/`OK`
  handshake exits back to Simple Serial (confirmed via
  `optoKummenberg.commands.Command.go_pro()`/`go_simple()`). The driver
  wraps every position read in enter/exit-Pro-mode so moves stay on the
  simpler Simple Serial path.
- **No stop/abort command exists anywhere** — confirmed absent from both
  the Operation Manual and every method on the SDK's `Command` class, not
  just undocumented. `wait_for_settle()`'s timeout-escalation path has
  nothing to fall back on; this is a genuine firmware limitation, not a
  gap in this driver.

`real_mr1530_motion.py` v3 implements all of this: `get_position()`,
`get_position_deg()`, `get_absolute_position_deg()`, `zero_here()`,
`calibration_jog()`, `calibration_jog_deg()`, and `home()`'s
`hard_home: false` fiducial path (now just calls `zero_here()`) are real,
not `NotImplementedError` stubs.

## 4. What's done now vs. what's still open

Done in `real_mr1530_motion.py` v3: everything from v2 (config parsing
including a hard refusal to start without `deg_per_mm_x/y` explicitly set,
serial connect + `START`/`GETID` handshake, `move_to()`, `wait_for_settle()`
via STATUS bit 4, `home()`'s `hard_home: true` path) **plus** the full
Pro-mode position-readback stack: `get_position()`, `get_position_deg()`,
`get_absolute_position_deg()`, `zero_here()`, `calibration_jog()`,
`calibration_jog_deg()`, and `home()`'s `hard_home: false` fiducial path.
Every `MotionController` method is now implemented — no
`NotImplementedError` stubs remain. `motion_controller.py`'s factory has
the `mr1530` branch wired in. `config.example.yaml` has a draft commented
block.

**The driver is functionally complete but has never touched real
hardware.** Still open:

- **No pre-hardware test harness yet.** The CONEX-AGAP integration was
  verified before hardware arrived using a hand-written fake-serial
  simulator implementing the documented ASCII protocol (see section 5).
  The equivalent for this driver — now needing to simulate both Simple
  Serial AND Pro-mode binary frames — hasn't been built yet. This is the
  natural next step before trusting v3 against anything, simulated or
  real.
- **No stop/abort command exists in firmware** (confirmed, not just
  undocumented — see 3b). **DECIDED 2026-09-05**: `wait_for_settle()`'s
  timeout path freezes (raises `AxisStateUnknown`, sends no further
  commands, requires a human to check the hardware) rather than
  auto-recovering by driving to a known-safe XY — auto-recovery means
  commanding a new move on top of an axis whose real state is
  unconfirmed, which is exactly what `AxisStateUnknown`'s contract in
  `motion_controller.py` exists to prevent, and "safe" is
  optics-dependent in a way this driver has no basis to assert on its
  own. What v4 adds is diagnosis, not recovery: the timeout path now
  makes a best-effort Pro-mode read of the board-fault register
  (`_REG_SYSTEM_STATUS_ERRORS`, 0x1007) via
  `_diagnose_timeout_fault()` and folds the raw value into the raised
  exception/log — see `real_mr1530_motion.py`'s module docstring V4
  UPDATE section and `tests/test_mr1530_safety.py`. Still no true
  stop/abort exists; if the *controller itself* (not just the mirror)
  ever wedges, the only real fix is a power cycle — deferred until
  there's actual evidence of that happening, not built preemptively.
- `calibrate_scan_area.py`'s CONEX-tuned numeric constants (unchanged from
  v1 of this plan, see below) — still deferred, not blocking.

## 5. Rollback / testing posture

Keep `real_conexagap_motion.py` and the `conex_agap` config path exactly as
they are — commented alongside, not deleted — same posture as the 8742 code
after its cutover. The CONEX-AGAP integration was verified pre-hardware
against a hand-written fake-serial simulator implementing the documented
ASCII protocol; the same approach now applies here, with the protocol fully
known as of v3 (both Simple Serial and Pro-mode binary) — building that
simulator is the next concrete step, and the only way to exercise this
driver before physical hardware arrives (lead time still unconfirmed as of
2026-08-18, see `optotune_mr1530_fsm_evaluation` memory).
