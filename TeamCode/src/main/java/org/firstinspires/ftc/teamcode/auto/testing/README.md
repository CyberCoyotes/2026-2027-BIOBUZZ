# Match Autos: notes for the coach

From Joel. I won't be at the event, so this is what you need to run these Autos, judge how far to trust them, and fix them in the pits.

## Status (read this first)

- These were written with AI help (Claude Code), in one long session on 2026-11-17 to 11-20.
- They **compile**, and every Auto was run start to finish in a **simulation** (a fake robot and clock) and checked with a **geometry script** for clearance from the hive, flowers and walls.
- **None of the 12 has run on a robot on a real field.** The flower stops and timing were tuned on our practice field using the "Mirror" test Autos in `testingOps/`. The real field may differ.
- **Recommendation:** run each Auto you might use in **STEP mode** at the event's practice field before matches. If there's no chance to test, use **`Shoot Park`** (or `Move Park`), not `3 Volley`.

## What's here

12 Autos: 4 start spots × 3 routines. All 12 run the same code in `BiobuzzAuto.java`, and each small file only selects the alliance, spot and routine.

| Routine | Does | Est. time | Match-ready? |
|---|---|---|---|
| **Move Park** | Start → our loading zone. LEAVE + PARK. | ~4 s | Yes, once checked on the field |
| **Shoot Park** | Shoot the 4 preloaded POLLEN, then park. | ~8–10 s | Yes, once checked. **Default choice.** |
| **3 Volley** | Shoot, flower A, shoot, flower B, shoot, park. | **~35 s** from D6/C1, less from D1/C6 | **No.** Runs past 30 s (see below). |

| Start spot | Tile / wall | Robot faces |
|---|---|---|
| **Blue D6** | D6, rear wall (our usual blue start, from `FieldConstants`) | down, at the hive |
| **Blue D1** | D1, audience wall, straight across from D6, about ½ in from the blue flower | up, at the hive |
| **Red C1** | C1, audience wall (our usual red start, from `FieldConstants`) | up, at the hive |
| **Red C6** | C6, rear wall, straight across from C1, about ½ in from the red flower | down, at the hive |

Driver Hub names follow the pattern **`<Spot> - <Routine>`**, for example `Blue D6 - Shoot Park`, listed under the **Auto** group. **The four `3 Volley` Autos have been hidden (`@Disabled`) since 2026-11-21, so only the 8 `Move Park` and `Shoot Park` Autos are on the list.** The list as it appears on the Driver Station is in `TeamCode/docs/biobuzz-driver-station-autos.md`. Issue #112 flags the four D1 and C6 Autos "Remove or fix heading" `[#112]`. Blue parks in the blue loading zone (F2), red in the red loading zone (A5). Both come from `FieldConstants.parkPose()`, the same as `AutoLeaveAndPark`.

**Fallbacks** (in the main teamcode folder, not this one): `AutoLeaveAndPark` (LEAVE + PARK from D6/C1; currently `@Disabled`), and `Drive Forward` (`AutoEmergencyLeave`, drives 24 inches measured by the Pinpoint, with a timed backup; no Pedro, LEAVE only) if Pedro or the Pinpoint is down. It does not know the alliance. Four copies of it do (issue #114): `Blue - Drive Forward 24in`, `Red - Drive Forward 24in`, `Blue - Drive Reverse 24in` and `Red - Drive Reverse 24in` (`AutoBlueDriveForward24` and so on). They are "dumb" Autos with no tile or grid reference: the robot drives 24 inches straight from wherever it starts, forward (back against a wall) or backward (intake against a wall), and tells the TeleOp the alliance. They do not hand over a heading, so field-centric drive is only right if the robot started facing down the field map on blue or up on red (see the header of any of the four files).

## Running one at the event

**Setup**
1. Select the Auto for **our alliance and the tile we're starting on.** Wrong alliance means it aims at the other hive and parks on the wrong side.
2. Robot touching the wall, centered in the tile, facing the hive, **4 POLLEN** loaded.
3. The **turret must point straight ahead when the robot is powered on.** Its zero is wherever it pointed at power-up.
4. Power on **at least 30 s before INIT** so the Limelight boots.
5. Press INIT and don't touch the robot afterward (INIT re-zeroes the Pinpoint).

**INIT screen should show:** our alliance, tile and routine at the top. `Mode: NONSTOP`, `Actions: ON`, `Hive visible: YES` (shooting routines only), and **no `NOT TUNED` warning**.

**Defaults are match settings.** With no buttons pressed it runs NONSTOP with shooting and intaking on. The gamepad 1 buttons during INIT are for **testing only**:

| Button | Effect |
|---|---|
| A | STEP mode: stops after every drive until A is pressed again. **Never in a match** (no one can press A during Auto, so the robot just stops). |
| B | NONSTOP (default) |
| X | Drive only: shots and picks become short pauses. Useful for checking paths without balls. |
| Y | Shooting + intaking on (default) |

**After every run**, the Driver Hub keeps a run log on screen. A photo of it is the fastest way to see what happened:
- `SHOT (locked at 1.2 s, 1250 RPM, 4 dips)`: normal shot. Dips ≈ balls fired.
- `locked at -1.0 s`: the camera never locked, so it fired from the pre-aimed turret angle.
- `SKIPPED: flywheel never reached speed` / `out of time` / `CUT SHORT`: self-explanatory.
- `Drives that timed out`: a drive took >8 s (blocked or stuck).

## Known risks

1. **3 Volley is too long.** Estimated ~35 s. A built-in cutoff skips or cuts short any shot or flower after **24 s** so it can still park, which means shot 3 will usually be skipped, and it may still miss the park. Don't run it in a match until a nonstop run on a real field comes in under ~28 s.
2. **Flower bar.** If the robot drives straight instead of curving or drifts off its path, the flower bar's UP position is wrong and it's dragging. Quick fix: set `USE_FLOWER_BAR = false` in `BiobuzzAuto.java` (only affects 3 Volley picks). Proper fix: tune the bar with `FlowerPickerAlpha`.
3. **Shaking at stops.** Pedro actively holding a stop made the robot shake on our practice field, so holding is off (`HOLD_AT_STOPS = false`): the drive motors just stop at each stop. If shaking shows up anyway, Foresight needs re-tuning on a fresh battery.
4. **The flower start spots (D1 / C6) are tight.** The robot starts about ½ in from the flower. Every path from there drives straight off the wall first, and the geometry check shows it never gets closer than where it started, but it has to be placed carefully: centered in the tile, square to the wall. From these spots the robot parks facing the way it started, and shot 3 in the 3 Volley is taken from a spot a little way out (not jammed against the flower).
5. **Shot 2 turret swing (3 Volley).** On 11/16 the turret limits changed from ±120° to −110° (right) / +180° (left). The hive is behind the robot at shot 2, so the turret now swings **left and stops at its +180° limit**, about 11° short of the hive (it can't go past the limit). The camera can't finish the aim from there, so shot 2 fires after the 3 s lock timeout, 11° off. Expect `locked at -1.0 s` for shot 2 in the run log. It may still score, since the hive forgives about 5–13°. This hasn't run on the robot: before using 3 Volley, check in STEP mode that the turret cables are OK at +180° and whether shot 2 balls go in.
6. **Field estimates.** Field size (141.5 in), loading-zone size and hive position come from Pedro's field image and the manual, not a tape measure (issue #55 in `FieldConstants`).

## Quick fixes in the pits

Everything adjustable is at the top of **`BiobuzzAuto.java`**. One change there applies to all 12 Autos.

| Symptom | Change |
|---|---|
| 3 Volley runs long | Lower `FEED_SECONDS` (3.0) and `FLOWER_PICK_SECONDS` (2.5) |
| Not all 4 balls leave | Raise `FEED_SECONDS` |
| Camera slow to lock, shot fires off-target | Raise `LOCK_TIMEOUT_SECONDS` (3.0) |
| Wants to park sooner | Lower `SKIP_ACTIONS_AFTER_SECONDS` (24) |
| Flower bar dragging | `USE_FLOWER_BAR = false` |
| Bar too late or too early at a flower | `FLOWER_A_BAR_DOWN_INCHES` (20) / `FLOWER_B_BAR_DOWN_INCHES` (0) |
| Not reaching into a flower / pushing it | `FLOWER_A_CREEP_INCHES` (5) / `FLOWER_B_CREEP_INCHES` (8) |
| Hits the hive or a flower on a drive | Note which drive (STEP mode), then adjust that path's stop or control points in the "Field spots" section |

Change one thing at a time, redeploy, and re-check in STEP mode.

## How the code is organized

- `BiobuzzAuto.java`: the complete Auto (an iterative OpMode state machine): Pedro paths, the shot sequence (same as `ShootAndParkAuto`), flower picking, the 24 s cutoff, step mode, telemetry. Not listed on the Driver Hub by itself.
- 12 thin subclasses, e.g. `AutoBlueD6ThreeVolley`:
  `super(VisionSubsystem.Alliance.BLUE, StartSpot.MAIN, Routine.THREE_VOLLEY);`
  `StartSpot.MAIN` = D6 (blue) / C1 (red). `StartSpot.FLOWER` = D1 (blue) / C6 (red), across the field in the same column, next to our flower.
  This is one level of `extends`. I know `CLAUDE.md` says no base classes. I did it this way because 12 copies of a ~1000-line file couldn't be kept in sync.
- **Field positions are written once, in blue coordinates.** Red is the 180° rotation about the field center (`x → 141.5−x`, `y → 141.5−y`, heading + 180°), the same rule as `FieldConstants.makePose()`. Not a mirror.
- Pedro's Follower owns the drive motors and the Pinpoint. `DriveSubsystem` and `PinpointSubsystem` aren't used.
- The practice-field test Autos in `testingOps/` (`ThreeVolleyMirrorAlpha`, `OneVolleyParkMirrorAlpha`, `TwoVolleyRearFlowerMirrorAlpha`, `TwoVolleyWallFlowerMirrorAlpha`, all `@Disabled`) were built for our practice-field layout. **They should not be run in a match.**
