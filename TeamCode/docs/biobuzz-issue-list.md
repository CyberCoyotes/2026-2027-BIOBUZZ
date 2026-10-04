# BIOBUZZ issue list (not yet on GitHub: Issues are disabled on the repo)

Types match the repo's issue types: Feature, Enhancement, Research.

## 1. Research: Confirm encoder counts per revolution on the robot (28 vs 112)
**Why:** Last season's notes say 112. goBILDA's spec says 28. Every RPM number in `ShooterSubsystem` depends on it.
**Evidence so far:** goBILDA's output-shaft encoder numbers (103.8 at 3.7:1, 537.7 at 19.2:1, 751.8 at 26.9:1) are each 28 x the ratio, so the base is very likely 28.
**How:** Plug in the encoders, run `ShooterTesting`, hold the **Right Trigger** (full power, hands clear), read `ticks/sec`. About 2,800 means 28. About 11,200 means 112.
**Done when:**
- [ ] Result written down (motor, battery voltage)
- [ ] If 112: both `TICKS_PER_REV` constants in `ShooterSubsystem` changed
- [ ] `[CONFIRM]` removed from `CLAUDE.md`, `ARCHITECTURE.md` section 8, and the `ShooterSubsystem` comment

## 2. Enhancement: Test ShooterSubsystem on the robot and tune tolerance and PIDF
**Why:** It compiles and passed a fake-motor simulation, but has never run on hardware. Several numbers are guesses.
**Steps:**
1. Set `flywheel_motor` and `backspin_motor` to the matching goBILDA motor type in the Driver Station configuration (believed 5203, 6000 RPM).
2. Do issue 1 first.
3. Check both RPM readings are positive when shooting.
4. Try Y / X / B / A presets. Watch `SPINNING_UP` then `READY`.
5. Tune one thing at a time: `AT_SPEED_TOLERANCE_RPM`, `AT_SPEED_HOLD_SECONDS`, `MAX_TARGET_RPM`, PIDF (`USE_CUSTOM_PIDF` only if the built-in values are slow or wobble), and the preset RPMs.
6. Check the belt ratio is really 1:1.

**Done when:**
- [ ] Both wheels reach each preset and `isAtSpeed()` goes true reliably
- [ ] State leaves `READY` right after a ball slows the flywheel, then recovers
- [ ] Tuned values written into the constants and `ARCHITECTURE.md` section 6

## 3. Feature: TurretSubsystem and TurretTesting with mag-switch re-zero
**Design:** approved and recorded in `ARCHITECTURE.md` section 6.
- Hardware: `turret_motor` (power only), `turret_encoder` (REV Through Bore Encoder, incremental, 8192 counts/rev, read only), `turret_home_switch` (REV magnetic limit switch, active-low).
- **0 degrees = home switch = intake direction.** Signed angles, **counterclockwise positive**. About +/-135 degrees of travel, dead zone behind. Soft limits about 5 degrees inside the range.
- Turret starts placed on the switch. INIT only warns if it is not. No motion at INIT (rules G304, G403).
- Switch turning on re-zeroes the angle. Re-arms after about 10 degrees of movement.
- Own P controller (D only if it oscillates), friction kick, max power clamp, `BRAKE` on stop.
- Methods: `setTargetAngle`, `setManualPower`, `stop`, `update`, `getAngleDegrees`, `isOnTarget`, `isZeroConfirmed`, `isHomeSwitchActive`, `getState`, `addTelemetry`.

**Blocked by:** issue 4.
**Safe first test order:** (1) turn by hand, watch ticks and switch state; (2) check switch polarity; (3) gentle manual moves with +/-45 degree limits, positive stick must increase the angle; (4) cross-check ticks per degree through a marked 90 degrees; (5) widen limits, try D-pad presets.
**Done when:**
- [ ] Code follows the Shooter pattern
- [ ] Verified on the robot
- [ ] `isOnTarget()` usable as the turret half of the firing gate

## 4. Research: Turret hardware facts (tooth counts, motor series, switch, inertia)
- [ ] Ring gear tooth count (believed 48T; both pinions are 16T). At 48T: about 68.3 encoder ticks per turret degree.
- [ ] goBILDA motor series: all believed to be 5203 (8mm REX shaft), not 5202. Check labels.
- [ ] Turret motor gearbox ratio (suggested total reduction about 60 to 100:1, for example 19.2:1 with a 3:1 ring and pinion).
- [ ] Moment of inertia of the rotating hood about the turret axis (Onshape Mass properties).
- [ ] Mag switch: confirm it is the REV Magnetic Limit Switch, where it mounts, approve the name `turret_home_switch`.
- [ ] Encoder: confirm the REV Through Bore Encoder is in incremental mode on an encoder port, configured as `turret_encoder`.
- [ ] What actually limits travel to 270 degrees? The ring looks like a full circle in the CAD.

**Done when:** values recorded in `ARCHITECTURE.md` section 6, `[CONFIRM]` markers removed, hardware names in the names table.

## 5. Feature: ShotTable (distance to flywheel and backspin RPM)
A small class separate from `ShooterSubsystem`. Measure 4 to 6 distances with `ShooterTesting`, store them as named arrays, interpolate, clamp outside the range. Distance comes from `d = (50 - lensHeight) / tan(cameraTilt + ty)`, minus about 2.7 in. Students must be able to retune it by editing the arrays.
**Blocked by:** issue 2 (and live distance needs issue 6).
**Done when:**
- [ ] Table built from real measurements
- [ ] Student-readable, no lambdas or streams
- [ ] Retune means editing arrays only

## 6. Feature: VisionSubsystem (Limelight AprilTag aiming data)
Wraps the Limelight 3A: valid flag, `tx`, `ty`, distance estimate.
- **Filter by alliance tag IDs** (the other hive is 25.5 in away).
- **Target the higher cluster** (larger `ty`).
- **Never use these tags for field position.** They move with the hive.
- Handle stale or missing data.
- Config: Ethernet device `limelight` at `172.29.0.1`; start with exposure 1000, gain 8.

**`[CONFIRM]` first:** Limelight 3A vs webcam with `VisionPortal`.
**Done when:**
- [ ] Reads `tx`, `ty`, valid; alliance filter works; higher cluster chosen
- [ ] Distance checked with a tape measure
- [ ] Exposure and gain retuned under gym lighting
- [ ] `ARCHITECTURE.md` section 6 updated

## 7. Feature: Fallback TeleOp (turret locked, 2 to 3 preset shots, no vision)
Build this first and keep it working all season. Turret locked at 0 degrees, driver picks a preset spot with fixed flywheel and backspin RPM, feed only when `shooterSubsystem.isAtSpeed()` (and the turret `isOnTarget()`), never hold more than 4 balls (G407).
**Blocked by:** issues 2 and 3, and a feeder/intake subsystem.
**Done when:**
- [ ] Driver can score from each preset spot with no Limelight
- [ ] Preset RPMs measured on the field and written as constants

## 8. Research: How many balls tip the hive, and what is the Limelight's max range?
1. Balls to tip the hive: not in the manual text. Check field CAD, a Team Update, or test. A tip is worth 20 points.
2. Limelight 3A max reliable range on a 3.25 in tag (36h11) under gym lighting. Expect trouble around 8 to 10 ft.
3. Verify on a real field: tag cluster center height (about 50 in, derived) and opening center height (about 59.5 in, derived).

**Done when:**
- [ ] Ball count in `biobuzz-game-essentials.md`
- [ ] Range and working exposure and gain in `biobuzz-hive-apriltag-geometry.md`
- [ ] "Verify on the real field" checklist ticked

## 9. Enhancement: Decide testing/ folder, main vs master, gold-standard subsystem
- [ ] Move the Alpha test OpModes (and `ShooterTesting`) into a `testing/` package?
- [ ] Main branch: `main` or `master`?
- [ ] Gold-standard subsystem class: `ShooterSubsystem` needs the coach's review to become the template.
- [ ] Vision source (also in issue 6).

**Done when:** each answer is written into `ARCHITECTURE.md` section 8.

## 10. Research: Evaluate a 360 degree turret
Would let us intake in one direction and shoot in the opposite one. In the CAD the ring gear looks like a full circle.
1. **Cables:** the hood carries two shooter motors and the Limelight. Continuous rotation needs a slip ring that handles motor current, or about +/-200 degrees with a cable loop. Ask mechanical. Not yet researched.
2. **Software:** needs angle wrap-around and shortest-path logic.
3. **Zeroing:** the mag switch still works, one reference per revolution.
4. The signed-angle convention carries over unchanged.

**Plan:** finish the 270 degree turret first, then decide.
**Done when:** mechanical has answered the cable question and the decision is recorded in `ARCHITECTURE.md`.

---

## Candidates not in the list yet
- Feature: `DrivetrainSubsystem` plus Tier 1 auto (LEAVE + PARK is 13 points and half of the SWARM ranking point)
- Feature: `IntakeSubsystem` / feeder (max 4 balls, G407)
- Enhancement: set up build flavors and Pedro Pathing (`ARCHITECTURE.md` section 5)
