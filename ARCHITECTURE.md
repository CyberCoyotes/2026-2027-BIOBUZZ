# Architecture — 2026-2027 BIOBUZZ

*This is the source of truth for how team code is structured. Claude Code and students read it before starting any coding session. `[CONFIRM]` marks a decision the coach hasn't made yet: **stop and ask** before doing work that depends on one. When a decision is made, replace the marker with the answer and mention the change in the commit message.*

## 1. Strategic context

Last season (DECODE) we used FTCLib command-based code. **This season we use the standard FTC SDK style instead**, for three reasons:

1. **It's what the FTC samples use.** Students can learn from `FtcRobotController/.../external/samples/` directly, without translating between two styles.
2. **It's easier to trace.** An iterative OpMode runs top to bottom every loop, with no scheduler deciding what runs when.
3. **It carries forward.** 2027-2028 moves FTC to SystemCore with more standard OpModes. Code habits built this year transfer.

We still organize the robot into **subsystems**, the same idea as in FRC: one class per mechanism. What we dropped is the framework (FTCLib's base classes, commands, and scheduler), not the idea. Our subsystems are plain Java classes.

## 2. MUST: non-negotiable conventions

### Where code lives

| Path | What goes there |
|---|---|
| `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/` | FTC's reference samples. **Read only.** |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` | Shared team code: test OpModes (and the subfolders below) |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/` | Subsystem classes: `ShooterSubsystem`, `TurretSubsystem`, … |
| `TeamCode/src/<flavor>/java/...` | Robot-specific OpModes (once flavors are set up; see section 5) |
| `TeamCode/docs/` | Game and robot reference docs |

### Package layout

```
org.firstinspires.ftc.teamcode
├── subsystems/     ShooterSubsystem, TurretSubsystem, IntakeSubsystem, DriveSubsystem, VisionSubsystem …
├── testing/        test OpModes (the *Alpha files)   [CONFIRM]
├── pedroPathing/   Pedro Pathing constants and tuning (per Pedro's setup guide)
└── (flavor dirs)   TeleOp and Auto OpModes for each robot
```

The `subsystems/` folder is decided. The existing test OpModes (`DriveAlpha`, `IntakeAlpha`, `ShooterPowerAlpha`, `ShooterRPMAlpha`, `ShooterRatioAlpha`, `TurretAlpha`, `VisionAlpha`, `ShotTableAlpha`) sit at the package root, and so does `BiobuzzTeleOp`, the combined TeleOp being built up one tested piece at a time (screen name "TeleOp - Alpha 7"; gamepad1 is the driver, gamepad2 is the operator, and every button always does the same thing, see its header). Moving them into `testing/` is a `[CONFIRM]`. Until that's decided, new test OpModes also go at the package root, next to the Alpha files.

### OpModes

- **Iterative OpModes** (`extends OpMode`), using `init()`, `init_loop()`, `start()`, `loop()`, and `stop()`.
- Start from the closest SDK sample (`BasicOpMode_Iterative`, `ConceptAprilTag`, …) and keep its section comments ("Code to run ONCE when the driver hits INIT", etc.).
- Always stop every motor in `stop()`.
- Put all tunable numbers at the top as `private static final` constants with a comment explaining how to change them. The Alpha test OpModes are the model.
- Annotate with `@TeleOp(name = "…", group = "…")` or `@Autonomous(...)`. Test OpModes use `group = "Testing"`.
- **Keep the Driver Station list short.** Once a subsystem is well tested, add `@Disabled` above its Alpha's `@TeleOp` line. Never delete the file. To bring it back, comment out the `@Disabled` line and redeploy. Currently hidden: `DriveAlpha`, `IntakeAlpha`. The shooter and turret Alphas stay visible while those subsystems are being worked on.

### Subsystem classes (one per mechanism)

Each subsystem is a **plain Java class**: no base class (no FTCLib `SubsystemBase`), no commands, and no scheduler. It follows the idea in the SDK's `ConceptExternalHardwareClass` sample, split by mechanism. The difference from an FRC subsystem is that nothing calls `periodic()` for you: the OpMode's `loop()` calls `update()`.

**Naming:** every subsystem class name ends in `Subsystem` (`ShooterSubsystem`), and OpMode class names never do. That way, anywhere you see a class name, you can tell whether it's an OpMode or a subsystem. Use the full name for the variable too (`shooterSubsystem`).

```java
package org.firstinspires.ftc.teamcode.subsystems;

public class ShooterSubsystem {
    // Constants: tunable numbers, ALL_CAPS
    // Hardware: private motor/servo/sensor fields

    public ShooterSubsystem(HardwareMap hardwareMap) { /* get hardware, set directions and modes */ }

    // Actions the OpMode calls: setTargetRpm(), stop(), …
    // Questions the OpMode asks: isAtSpeed(), getRpm(), …

    public void update() { /* called once per loop() if the subsystem needs it */ }
    public void addTelemetry(Telemetry telemetry) { /* shows its own data */ }
}
```

Rules:
- **A subsystem owns its hardware.** Only `ShooterSubsystem` touches the flywheel motors, only `TurretSubsystem` touches the turret motor, and so on. OpModes call subsystem methods; they don't reach into the motors.
- **Constructor = setup.** It gets hardware from the `HardwareMap` and sets directions, modes, and zero-power behavior.
- **`update()` runs once per loop** for any subsystem with ongoing work (velocity checks, turret aiming, timers). The OpMode calls it; nothing calls it automatically.
- **State vs. status:** *state* is what the code told the subsystem to do (`targetRpm`, an enum like `IDLE / SPINNING_UP / READY`). *Status* is what sensors report (`getRpm()`, `hasTarget()`). Keep them separate so telemetry can show both.
- **No button references inside subsystem classes.** Gamepad handling lives only in TeleOp OpModes.
- **Sensors-only devices** (Limelight, Pinpoint) get plain wrapper classes too, e.g. `VisionSubsystem`, with an `update()` that reads the latest data once per loop.

### Where constants live

In FRC a `Constants` file is common. In FTC we keep each number with the class that owns it, because one file then explains the whole mechanism and a student never has to hunt for it. Ask "what does this number describe?":

| The number describes… | It lives in… | Examples |
|---|---|---|
| The **mechanism** | That mechanism's subsystem class, as `private static final` at the top | `DEFAULT_INTAKE_POWER`, motor directions, `FLYWHEEL_MAX_TARGET_RPM`, servo positions, soft limits |
| How a **test OpMode drives** a mechanism | That test OpMode | Dial step size, the Y/X/B/A preset powers, nudge step |
| Something **two or more classes** need, or the **field or robot** itself | A shared constants class (see below) | Hive and tag dimensions, camera and turret offsets |

Rules:
- **Don't copy a number into two files.** When a test OpMode finds a good value (the best intake power, a servo position), copy it into the subsystem's constant. The OpMode does not keep the real value, or the competition TeleOp would never see the tuning.
- **Hardware names live only in the subsystem that owns the hardware.** OpModes never call `hardwareMap.get`, so each name has exactly one home.
- **A shared constants class is created only when a second class needs the value.** Do not make a constants file with empty sections "just in case". Name it for what it holds (`FieldConstants`, `RobotConstants`). **Never call it plain `Constants`**: Pedro Pathing's setup puts its own `Constants` class in `pedroPathing/`, and two classes with the same name are confusing to import.
- **Per-robot values** (practice bot, 11940, 22091 will differ) wait until the build flavors are set up (section 5).

### Typical TeleOp shape

```java
@TeleOp(name = "BIOBUZZ TeleOp", group = "Competition")
public class BiobuzzTeleOp extends OpMode {
    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;

    @Override public void init() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
    }

    @Override public void loop() {
        // 1. Read sensors (vision.update(), odometry, …)
        // 2. Read gamepads and decide what to do
        // 3. Tell subsystems what to do, then call their update()
        // 4. Telemetry
    }

    @Override public void stop() {
        shooterSubsystem.stop();
        driveSubsystem.stop();
    }
}
```

Keep that four-step order in every TeleOp `loop()`.

### Autonomous

- **Pedro Pathing** for paths. Follow Pedro's example pattern: an iterative OpMode with `follower.update()` in `loop()` and a `pathState` switch that moves from step to step.
- Auto uses the **same subsystem classes** as TeleOp, **except the drivetrain**. Pedro's `Follower` drives the four drive motors in Auto, and only one class may control a motor, so `DriveSubsystem` is for TeleOp and Pedro drives in Auto. Never use both in one OpMode. Pedro's motor names and directions in its `Constants` must match ours. If we later want Pedro path-following inside TeleOp (a lower priority), `DriveSubsystem` would wrap the `Follower` and keep the same public methods, so TeleOp OpModes would not change.

### Naming

| Thing | Convention | Example |
|---|---|---|
| Packages | lowercase | `subsystems` |
| Classes (OpModes and others) | PascalCase | `ShooterPowerAlpha` |
| Subsystem classes | PascalCase, ends in `Subsystem` | `TurretSubsystem` |
| Variables / methods | camelCase | `targetVelocity` |
| Hardware variables | location first | `leftIntakeServo` |
| Hardware config names | snake_case | `left_intake_servo` |
| Constants | ALL_CAPS | `MAX_TURRET_DEGREES` |
| Doc filenames | kebab-case | `biobuzz-robot-design.md` |

### Hardware config names (robot configuration on the Driver Station)

Names already in use:

| Name | Device |
|---|---|
| `intake` | One motor for intake, index, and transfer (goBILDA 5203, probably 1620 RPM 3.71:1 `[CONFIRM]`) |
| `flywheel` | Shooter flywheel: goBILDA 5203 series **1620 RPM (3.7:1)**, about 4 in, on the front of the ball path. In the Driver Station config set its motor type to match (see `biobuzz-robot-config-quick-reference.md`) |
| `hood_wheel` | Hood wheel: goBILDA 5203 series 6000 RPM (1:1; spec sheet `5203-2402-0001` in `TeamCode/docs/`), about 2 in, on the back/top side in the fixed hood. Its RPM relative to the flywheel sets the ball spin. |
| `left_front` | Drive motor, left front (mecanum) |
| `left_rear` | Drive motor, left rear (mecanum) |
| `right_front` | Drive motor, right front (mecanum) |
| `right_rear` | Drive motor, right rear (mecanum) |
| `turret_motor` | Turret rotation motor (goBILDA 5203 series, 435 RPM, 13.7:1; spec sheet `5203-2402-0014` in `TeamCode/docs/`). The REV Through Bore Encoder (incremental/quadrature, on the turret ring gear) is plugged into this same port's encoder connector, so there is no separate encoder entry. |
| `turret_home_switch` | REV magnetic limit switch (active-low) that re-zeros the turret angle (digital port). Name approved. |
| `shooter_servo` | Shooter gate: goBILDA dual mode 300° torque servo, set to positional mode. Moves about 90° between LOADING and SHOOTING. |
| `limelight` | Limelight 3A (Ethernet device) |

Add new names here when you add hardware. Names in code must match the configuration exactly.

### Dependencies

- Pedro Pathing is the only planned outside library. **Ask before adding any other dependency.**
- **Do not modify:** anything under `FtcRobotController/`, the Gradle wrapper, or root-level Gradle files, unless the task explicitly says so.

## 3. MUST NOT: anti-patterns

- No FTCLib, no commands, and no schedulers. No base class like `SubsystemBase`. (Plain subsystem classes are fine and expected.)
- No lambdas, streams, or anonymous classes. Write the loop out.
- No `sleep()` or blocking waits inside `loop()`. Use `ElapsedTime` timers instead.
- No gamepad reads inside subsystem classes.
- No two classes controlling the same motor.
- No magic numbers buried in code. Make every tunable value a named constant.
- Don't use the BIOBUZZ AprilTags to locate the robot on the field. They move with the hive; use Pinpoint for position and the tags only for aiming (see `TeamCode/docs/biobuzz-hive-apriltag-geometry.md`).

## 4. Reference code

- **Style models:** `IntakeAlpha.java` and `ShooterPowerAlpha.java`. They show the comment style, the constants-at-the-top pattern, and iterative OpMode structure.
- **Gold-standard subsystem class:** `[CONFIRM]`. The first subsystem class written and approved becomes the template for the rest. `ShooterSubsystem` is the first one planned.
- **SDK samples most likely to be useful:** `BasicOpMode_Iterative`, `ConceptExternalHardwareClass` + `RobotHardware`, `ConceptAprilTag`, `SensorLimelight3A`, `SensorGoBildaPinpoint`, `RobotAutoDriveToAprilTagOmni`.

## 5. Setup still needed

- [ ] Port the **build flavors** (`practiceBot`, `team11940`, `team22091`) from last season's `TeamCode/build.gradle`. They're not in this repo yet.
- [ ] Add **Pedro Pathing** to the Gradle files and set up its constants/tuning.
- [ ] `[CONFIRM]` Which git branch is the main one: `main` or `master`? The repo has both, and the claude.ai Project syncs `master`.

## 6. Hardware and behavior

The robot is still being designed. See `TeamCode/docs/biobuzz-robot-design.md` for the concept (turret shooter, hood wheel, Limelight aiming) and its open decisions. Add each subsystem's hardware and behavior to this section once its design is approved.

### ShooterSubsystem (`subsystems/ShooterSubsystem.java`)

- **Hardware:** `flywheel` is a goBILDA 5203 series **1620 RPM (3.7:1)** motor (changed from 6000 RPM on the `change-shooter-motors` branch). `hood_wheel` is still a goBILDA 5203 series 6000 RPM (1:1) motor, confirmed by the coach. Both have encoders plugged in, belt driven and believed to be 1:1 `[CONFIRM]`.
- **Different motors, different numbers (not tested on the robot yet):** `ShooterSubsystem` now has a ticks per rev, a motor top speed, and a max target RPM **for each wheel**. Flywheel: **103.8** ticks per output rev (spec sheet `5203-2402-0003`: 28 at the encoder shaft, 103.8 at the output shaft), 1620 RPM top speed, **1458 RPM max target** (90%). Hood wheel: 28 ticks per rev (spec sheet `5203-2402-0001`), 6000 RPM, 5400 RPM max target. `getMaxTargetRpm()` was replaced by `getMaxFlywheelTargetRpm()` and `getMaxHoodWheelTargetRpm()`. The encoder is on the motor shaft (before the gearbox), so the speed at full power is about 2,800 ticks/sec on **both** motors. The numbers are not in doubt (they come from the spec sheets), but a full-power run in `ShooterRPMAlpha` is a quick sanity check for wiring and config mistakes: the hood wheel should read about 6,000 RPM and the flywheel about 1,500 to 1,620 RPM.
- **Hood wheel ratio:** `HOOD_WHEEL_RATIO` is **2.0** (hood wheel RPM / flywheel RPM), the best value found in robot testing so far. `setTargetRpm()` uses it, and `BiobuzzTeleOp` starts its ratio dial at the same number. The **flywheel** now hits its limit first: it tops out at **1458 RPM** (`FLYWHEEL_MAX_TARGET_RPM`), where the hood wheel only needs 2916 RPM, far under its own 5400 RPM limit.
- **Control:** speed control in RPM (`setTargetRpm()` / `setTargetRpms()`). `setPower()` is open loop, for tests and clearing jams. `stop()` coasts (never commands a speed of 0).
- **States:** `IDLE`, `OPEN_LOOP`, `SPINNING_UP`, `READY`. `isAtSpeed()` is true only in `READY` and is the shooter's half of the firing gate. The turret's on-target check is the other half.
- **Test OpMode:** `ShooterRPMAlpha`. Its Right Trigger button runs the full-power encoder sanity check (see above).
- **Ratio test OpMode:** `ShooterRatioAlpha` finds the best `HOOD_WHEEL_RATIO` (hood wheel / flywheel). **It uses speed control (RPM)**, now that the flywheel and hood wheel encoders are connected. B/A turn the flywheel RPM dial (100 RPM steps), D-pad Left/Right turn the ratio dial (0.05 steps), Y/X start and stop the shooter, and hood wheel RPM = flywheel RPM x ratio. It warns CAPPED when the hood wheel target is over `ShooterSubsystem.getMaxTargetRpm()`, shows the measured ratio, and records the lowest RPM each wheel hit after reaching speed (Back resets it). Printable student sheet: `TeamCode/docs/hood-wheel-ratio-test-sheet.html` (and a `.pdf` copy). The old plain-POWER version is saved in git (commit `88c7669`), with its sheet `hood-wheel-ratio-test-sheet-power.html`, in case the encoders ever stop working.
- **At-speed tolerance is 25 RPM** (was 100) because the shooter gate will only open on `isAtSpeed()`. **Both** wheels must be within it, and one reading outside it drops READY at once. `[CONFIRM]` on the robot: if the RPM readout wanders more than about 15 RPM while steady, READY will flicker. Then fix the speed reading or the PIDF, don't loosen the tolerance without asking.
- **RPM smoothing (not tested on the robot yet):** the Hub's speed reading jumps about ±20 RPM from loop to loop at 1300 RPM (a heavy flywheel does not really change speed that fast, so the jump is mostly reading noise). `update()` now blends each reading into a running average (an exponential low-pass filter, `RPM_FILTER_TIME_CONSTANT_SECONDS`, starts at **0.10**). `getFlywheelRpm()`, `getHoodWheelRpm()`, the READY check and the screen RPM use the smoothed value. The raw ticks/sec lines on the screen are **not** smoothed. Tradeoff: a bigger number is steadier but can hide or delay the real dip when a ball goes through, so lower it if READY stays on during a shot. Set it to 0.0 to turn smoothing off.
- **Starting guesses to tune on the robot:** the hold time (`AT_SPEED_HOLD_SECONDS`), `FLYWHEEL_MAX_TARGET_RPM` and `HOOD_WHEEL_MAX_TARGET_RPM`, the PIDF numbers (the Hub's built-in values are used until `USE_CUSTOM_PIDF` is turned on), and the preset RPMs in `ShooterRPMAlpha`.
- **Not in this class:** the distance-to-RPM shot table (a separate class later), feeding, and anything that reads a gamepad.

### ShooterServoSubsystem (`subsystems/ShooterServoSubsystem.java`): code written, not yet tested on the robot

- **Hardware:** `shooter_servo`, a goBILDA dual mode 300° torque servo. It acts as a **gate** and moves about 90° between two positions. Set it to **positional** mode with the goBILDA Servo Programmer, and configure it as a plain "Servo" on the Driver Station.
- **Separate from `ShooterSubsystem`** so the flywheel tests still run on a robot where the servo is not configured yet.
- **States:** `LOADING` and `SHOOTING`. A servo reports no position, so this is only what we commanded. `isAtPosition()` is a **time guess** (`SERVO_TRAVEL_SECONDS` since the last command), and is the gate's half of the future firing gate.
- **Methods:** `goToLoading()`, `goToShooting()`, `toggle()`, `nudgePosition()` (for finding positions), `getState()`, `isAtPosition()`, `addTelemetry()`. No `update()`, because there is no ongoing work. It starts in `LOADING` when constructed (R103.B allows pre-positioning servos at INIT).
- **Test OpModes:** both `ShooterPowerAlpha` and `ShooterRPMAlpha`. **Left Trigger** toggles the gate (one flip per press). **D-pad Up/Down** nudges the current position by 0.01.
- **`[CONFIRM]` positions are TBD:** `LOADING_POSITION` (0.30) and `SHOOTING_POSITION` (0.0, the end of the servo's travel) are placeholders. Find the real values with the D-pad nudge, then type them into the constants. Also tune `SERVO_TRAVEL_SECONDS` (0.50 is a guess).
- **Open:** which end of travel is "closed", and whether the gate needs a different position in Auto.

### TurretSubsystem (`subsystems/TurretSubsystem.java`): code written, not yet tested on the robot

- **Hardware:**
  - `turret_motor`: goBILDA 5203, **435 RPM (13.7:1)**, power on the port's power connector.
  - **For now the turret uses the motor's built-in encoder** (**384.5 counts per output-shaft revolution**, which is 28 × 13.7, per spec sheet `5203-2402-0014`), plugged into the port's encoder connector. **Later** the REV Through Bore Encoder (incremental/quadrature, **8192 counts per revolution**, on its own 16T pinion on the ring gear) can go in that same connector instead; change `ENCODER_TICKS_PER_REV` in `TurretSubsystem` when it does. There is **no separate `turret_encoder` config entry**: we have 8 motors and the two hubs have exactly 8 motor ports. The code reads it with `turretMotor.getCurrentPosition()`. The Control Hub cannot read the Through Bore's absolute pulse output.
  - **Direction coupling:** because they share a port, `setDirection()` reverses the motor and the encoder count together. `INVERT_ENCODER_READING` fixes the encoder's sign in software.
  - `turret_home_switch`: REV magnetic limit switch. It is **active-low**: it reads LOW when a magnet is near. Confirm on the robot with telemetry. **Configure it as a "Digital Device"** in the Driver Station, not "REV Touch Sensor": the SDK's touch-sensor class is not a `DigitalChannel`, which is what our code reads.
- **Gearing:** both spur gears (motor pinion and encoder pinion) are 16T. The ring gear is believed to be 48T `[CONFIRM]`. So the encoder shaft turns 3 times per turret revolution, which is `ticks per rev × (ring teeth / 16) / 360`. With the motor's built-in encoder (384.5) that is about **3.2 ticks per turret degree** (0.31° per tick). With the Through Bore (8192) it would be about 68.3. Cross-check by turning the turret by hand through a marked 90°.
- **Angle convention:**
  - **0° is the home switch, and it points at the intake direction** (straight ahead).
  - Angles are **signed**, and the travel is about 270° (±135°). The 90° dead zone is directly behind, opposite the intake.
  - **Positive is counterclockwise viewed from above** (turning left). This matches Pinpoint and Pedro Pathing heading, so `shooterFieldHeading = robotHeading + turretAngle`.
  - Soft limits are **±120°**, 15° inside the range, so the turret has room to coast before the cables or frame.
- **Zeroing:**
  - At power-up the turret is placed by hand **on the switch**. INIT telemetry warns if the switch is not active. There is no motion at INIT (G304, G403).
  - During a match, the switch turning on resets the encoder count to zero (0°). To avoid repeated re-zeroing while hovering at the magnet's edge, it re-arms only after the turret has moved about 10° away.
  - The hub keeps encoder counts between Auto and TeleOp, so nothing is stored in a static field. `isZeroConfirmed()` is false until the switch has been crossed once.
- **Control:** our own P controller (D added only if it oscillates) on the angle error, with a small friction kick, a max power clamp, and soft limits that cut power pushing past a limit. **Feedforward:** `setFeedforwardPower(power)` adds an extra push on top of position control so the turret can follow a spinning robot (position control only pushes once the turret is already off target, so on its own it always trails a spin). It **lasts one loop** (`update()` clears it, so a stale value can never keep the turret moving), it also works inside the "close enough" band, and the max power and soft limits still apply to the total. If you turn `KD` on while using it, `KD` pushes against the following motion. The motor `FLOAT`s until the first command (so the turret can be placed on the switch by hand during INIT), then uses `BRAKE` to hold position. `stop()` floats again.
- **Soft limits are ON at ±120°** (`SOFT_LIMITS_ON = true`, `MIN_ANGLE_DEGREES` / `MAX_ANGLE_DEGREES` in `TurretSubsystem`). They cut power that pushes past a limit and clip `setTargetAngle()`. They are only as good as the angle: with no home switch, 0° is where the turret pointed at power-up, so power on pointing straight ahead. Still check the encoder direction and ticks per degree on the robot first (step 1 and 2 in `TurretAlpha`).
- **Test controls:** `TurretAlpha` uses **gamepad2**: left stick and D-pad Left/Right turn the turret (hold to move, let go to hold the angle), D-pad Up/Down change the turret power by 5% per press (default 100%), **Y / X / B** go to the center / far left (+90°) / far right (−90°) presets, **A** stops. Stick right = turn right, which is a *negative* angle, so `TurretAlpha` flips the stick sign. `MAX_MANUAL_POWER` in the subsystem is 1.0, so the OpMode's power dial is the real motor power.
- **Speeds (100%):** the manual stick power is 100% in `BiobuzzTeleOp` and `VisionAlpha` (and the `TurretAlpha` dial starts there). Position control (auto-align and the presets) is capped by `MAX_POWER = 1.0` in `TurretSubsystem`. Both were 50% for the first tests, then 75%. Stopping distance grows with the square of the speed, so at 100% the turret overshoots more and coasts further past a soft limit. **Not yet tested at 100%:** re-check the limit coasting (ease the stick into a limit first) and auto-align overshoot. The ±120° limits only tested safe at 50%.
- **Tested on the robot:** the ±120° limits, and `VisionAlpha` auto-align (`tx` falls toward 0 when the turret turns toward the target).
- **Test OpMode:** `TurretAlpha` (gamepad2: manual stick, hold-to-turn D-pad, power dial, Y/X/B presets, A to stop). Its header has the safe first-test order.
- **Open:** exact ring tooth count, whether the switch triggers at a different angle depending on direction of travel, and whether we go to a 360° turret.

### VisionSubsystem (`subsystems/VisionSubsystem.java`): code written, not yet tested on the robot

- **Hardware:** `limelight`, the Limelight 3A, as an Ethernet device (USB-C to USB-A into the Control Hub's USB 3.0 port). It needs about 30 seconds to boot.
- **Reads only.** It never moves the turret and has no gamepad code. The OpMode takes its answers and tells the turret what to do.
- **Target choice:** keeps only our alliance's tags (RED = tags 30-37, BLUE = tags 38-45), picks the cell holding the **highest** tag (largest `ty`, which is the raised cell), then **averages `tx` and `ty` over the visible tags of that one cell**. Averaging avoids aiming up to 6.5 in off center at a single tag.
- **Sign convention:** `getTxDegrees()` is the Limelight's: **positive = target to the RIGHT**. Turret angles are positive to the LEFT, so aiming code uses `turretAngle - tx`.
- **States:** `NO_TARGET`, `TARGET_VISIBLE`. A result older than `MAX_RESULT_AGE_MS` counts as no target.
- **Methods:** `setAlliance()`, `getAlliance()`, `update()` (once per loop), `hasTarget()`, `getTxDegrees()`, `getTyDegrees()`, `getTagCount()`, `getTargetCell()`, `getResultAgeMs()`, `isCameraConnected()`, `stop()`, `addTelemetry()`.
- **Test OpMode:** `VisionAlpha` ("Vision - Alpha"). Step 1 is camera only (watch the numbers in INIT). Step 2 is auto-align: gamepad2 Right Bumper held turns the turret to cancel `tx`, Left Stick X is manual and wins, A stops. Gamepad1 **START** switches the alliance (starts **BLUE**). It picks a new turret target only when `|tx|` is over 1 degree, so the turret's on-target check can settle. If the target is lost it holds; it does not search.
- **Aim offset (the fudge factor):** `AIM_OFFSET_DEGREES` (starts at 0.0) is added to the camera's tx inside `update()`, so `getTxDegrees()`, the "aimed" check, and auto-align all get it. **Positive = the turret aims a little more to the RIGHT, negative = more to the LEFT.** Tune it by shooting from one fixed spot: shots landing right of the hole means a more negative number, left means more positive. Change it by about 0.5° at a time (1° is about 1 inch of miss per 5 ft). It is not applied when there is no target. Telemetry shows the raw camera tx next to it.
- **Starting values to check on the robot:** `APRILTAG_PIPELINE_INDEX` (0, must be an AprilTag pipeline in the Limelight web page), `MAX_RESULT_AGE_MS`, `AIM_OFFSET_DEGREES`, and the two thresholds in `VisionAlpha`.
- **Tested on the robot:** the camera reads tags (pipeline 0, cell 2, 3 tags used, 5 ms old), and `tx` falls toward 0 when the turret turns toward the target, and Right Bumper auto-align works.
- **`[CONFIRM]`:** the arm can hide a pair of tags. If it hides only the left or right pair, the average shifts toward the visible side. Measure how big that error is on the real hive. Camera latency (a few frames) can also make fast turret moves overshoot a little.
- **Not in this class:** a distance calculation (the shot table uses `ty` directly, see `ShotTable`), the vertical offset from the tags to the cell opening, and searching for a lost target.

### ShotTable (`subsystems/ShotTable.java`): code written, not yet measured on the robot

- **What it is:** a plain class with **no hardware**, no `update()`, and no gamepad code. You give it a `ty` and it returns the **flywheel RPM** for a shot from there. It is not a mechanism, so its name does not end in `Subsystem`. It sits in `subsystems/` because its numbers describe the shooter `[CONFIRM]`.
- **Why `ty` and not distance:** the camera is fixed on the turret, so a closer hive shows **higher** in the view, which is a **bigger `ty`**. That makes `ty` a stand-in for distance with no lens height or camera tilt needed. **If the camera is moved or re-tilted, measure the table again.**
- **The table:** two lists in the same order, `TY_DEGREES` (smallest to biggest, which is far to close) and `FLYWHEEL_RPM`. Between two rows it draws a straight line (linear interpolation). Outside the table it **uses the nearest row and never extrapolates**. The constructor stops with a clear message if the lists are different lengths or `ty` does not strictly increase.
- **Methods:** `getFlywheelRpm(ty)`, `isInRange(ty)`, `getRowCount()`.
- **Hood wheel:** the table has one RPM per row. The hood wheel is `flywheel × HOOD_WHEEL_RATIO` when the RPM goes through `ShooterSubsystem.setTargetRpm()`. A second column is an easy later change if testing shows a fixed ratio is not enough.
- **Speed cap:** the flywheel is the 1620 RPM motor, so `FLYWHEEL_MAX_TARGET_RPM` is **1458**. **Keep every flywheel RPM at 1458 or below.** Above that the flywheel is cut off and the hood wheel (twice what you asked) is off ratio. **The farthest spot (1425 RPM) is within 33 RPM of the cap, and these rows were measured with the old 6000 RPM flywheel motor, so recheck them on the robot.**
- **Test OpMode:** `ShotTableAlpha` ("Shot Table - Alpha", gamepad1). Minimal on purpose: no drive, turret, or gate. Move the robot **by hand** and load balls into the intake. Right Trigger shoots at a dial RPM (D-pad Up/Down) to find the slowest speed that scores; A saves a row (`ty` + RPM) to the screen; Left Trigger shoots at the table's RPM to check it; Left Bumper reverses both wheels and the intake; START switches alliance. **Both shoot triggers also run `IntakeSubsystem`, but only once `ShooterSubsystem.isAtSpeed()` is true** (`INTAKE_WAITS_FOR_SPEED`), so each ball is fed at the speed being measured. A ball slows the wheels, the intake pauses until they recover, and balls go one at a time. The gate servo is not touched. Nothing is saved to disk, so copy the rows into `ShotTable` before stopping.
- **`[CONFIRM]` first-pass rows are in:** 5 spots measured with `ShotTableAlpha` (7 saves, repeats merged), from `ty` -16.8 (1425 RPM; 1400 made the shot and 1450 felt more comfortable) to `ty` -4.0 (1250 RPM). `ty` is negative on our camera; the rule that matters is that a bigger `ty` is closer and needs a lower RPM. The RPMs are only good to about ±50 (the dial's step). Check the table with Left Trigger at a spot that was not measured, and add rows where it misses.
- **Wired into `BiobuzzTeleOp` ("TeleOp - Alpha 7"), not yet tested on the robot.**
  - **Vision joins the TeleOp.** It creates a `VisionSubsystem` and calls `update()` once per loop. **Every robot's Driver Station config must have the `limelight` Ethernet device**, or the TeleOp stops at INIT.
  - **Auto-align** (same logic as `VisionAlpha`: new turret target = angle now − `tx`, re-aimed only when `|tx|` is over 1°) **runs all the time (ALWAYS-TRACK, new in Alpha 7)**. `alwaysTrack` starts ON (`ALWAYS_TRACK_AT_START`) and never runs during INIT. **gamepad2 Right Stick Button** switches it ON / OFF, and gamepad2 Y, X, B and A (presets and STOP) switch it OFF so the turret stays where they put it. The **driver's Right Trigger still aligns AND shoots together**, even when ALWAYS-TRACK is OFF. gamepad2 Right Bumper keeps its old job (reverse the flywheel only). The turret stick and A (STOP) beat auto-align. With no target it holds the current angle. It does not search. **`[CONFIRM]`: the alliance starts as BLUE, so a red-alliance robot tracks the wrong hive until the operator presses Back. Decide whether the starting alliance should be set per robot or per match.**
  - **Following the robot's spin (feedforward), not yet tested on the robot:** while auto-aligning, the TeleOp calls `turretSubsystem.setFeedforwardPower(-driveSubsystem.getTurnPower() × TURRET_FEEDFORWARD_GAIN)` every loop. Robot spins left, turret is pushed right. While the robot is turning it picks a new turret target **every loop** (otherwise position control would pull the turret back toward an old target). The push only happens while a target is visible. `TURRET_FEEDFORWARD_GAIN` starts at **0.30**, a safe guess that follows a little less than needed. **Tune it on the robot:** spin steadily while aligned and watch `tx`. If `tx` drifts to the side the robot is spinning, raise the gain. If it swings the other way, lower it. Change it about 0.05 at a time. The screen shows "Turret follows spin". Checked only in a simple simulation (the feedforward cut the lag in every case, and a too-high gain made the turret lead instead of going unstable). The simulation's turret and robot speeds were assumptions, not measurements.
  - **Speed source:** gamepad2 **Start** switches **DIAL** (the operator's dial, as before, and the default) and **TABLE** (the table's RPM while the camera sees the hive). In TABLE mode, if the camera loses the hive, it **keeps the last table RPM for `TABLE_RPM_HOLD_SECONDS` (1.0 s), then goes back to the dial**. The screen says which one it is using. The hood wheel is always flywheel × the ratio dial.
  - **Alliance:** gamepad2 **Back** switches BLUE/RED (starts BLUE) and the alliance is always on the screen. The wrong alliance aims at the other hive, which is not an obvious failure.
  - **The dial now starts at 1300 RPM** (it was 2700, far too fast), so the fallback is sensible.
  - **Checked** by compiling and by running the real loop against fake subsystems (speed source, the 1.0 s hold, align priorities, alliance and mode toggles). Not run on the robot.
  - **Not in this session:** always-tracking (hold-to-align only), and gating the intake or gate on `isAtSpeed()` and "turret aimed".

### DriveSubsystem (`subsystems/DriveSubsystem.java`): code written, not yet tested on the robot

- **Hardware:** four drive motors named `left_front`, `left_rear`, `right_front`, `right_rear` (the same wiring on the practice bot, 11940 and 22091). The Driver Station config on each robot must use these names (one-time edit).
- **Owns the motors directly** for TeleOp, in power mode (`RUN_WITHOUT_ENCODER`). Pedro drives them in Auto (see Autonomous above).
- **Convention (matches Pedro Pathing, Pinpoint and the turret):** `drive(forward, strafe, turn)`. **Forward is positive, strafe LEFT is positive, turn COUNTERCLOCKWISE is positive.** Each input is -1.0 to 1.0. The stick signs are flipped in `DriveAlpha`, with a comment saying why.
- **Wheel math:** leftFront = forward - strafe - turn, rightFront = forward + strafe + turn, leftRear = forward + strafe - turn, rightRear = forward - strafe + turn. If any wheel would pass 1.0, all four are scaled down together.
- **Methods:** `drive`, `setSlowMode(boolean)` (slow mode; the 40% scale is `SLOW_MODE_POWER_SCALE` in the subsystem), `getTurnPower()` (how hard the robot is being told to spin, -1.0 to 1.0, + is counterclockwise, after slow mode; the TeleOp uses it to tell the turret how fast to follow the spin), `setWheelPowers` (raw, used by `drive` and the wheel tests), `stop`, `addTelemetry`. No `update()`, because there is no ongoing work.
- **Zero-power behavior:** `BRAKE` (a named constant), for precise stops. The robot is tall, so watch for tipping.
- **Robot-centric only** in the first version. Field-centric comes later, with a Pinpoint wrapper that supplies the heading.
- **Test OpMode:** `DriveAlpha` (sticks drive; hold the right trigger for slow mode; X, A, Y and B each spin one wheel for the direction check).
- **Starting values to check on the robot:** the direction constants (left motors `REVERSE`, right motors `FORWARD`, copied from the FTC sample), the slow-mode scale (40%), and `BRAKE`. Do the wheel test with the robot on blocks first.

### IntakeSubsystem (`subsystems/IntakeSubsystem.java`): code written, not yet tested on the robot

- **Hardware:** `intake`, one goBILDA 5203 series at 1620 RPM (3.7:1, believed `[CONFIRM]`). The one motor spins the front intake **and** a second hex shaft with grippers in the transfer chute, so intake and transfer always run together.
- **Control:** plain power, not RPM (`RUN_WITHOUT_ENCODER`). There is **one power dial** (`setIntakePower()`, 0.0 to 1.0, starting at 50%). `intake()` runs at +dial, `reverse()` runs at -dial, `stop()` coasts (`FLOAT`). Splitting reverse into its own number is an easy later change.
- **States:** `STOPPED`, `INTAKING`, `REVERSING`.
- **Encoder:** plugged in for testing. `update()` reads `getVelocity()` once per loop and `getTicksPerSecond()` returns it. It is shown in **raw ticks per second only, with no RPM conversion**, so nothing here depends on the ticks-per-rev number. (For the 1620 RPM motor it would be 103.8 per output rev, per spec sheet `5203-2402-0003`.)
- **Test OpMode:** `IntakeAlpha` (Right Bumper = intake, Left Bumper = reverse and wins, D-pad Up/Down = dial +/-5% per press). The D-pad dial is for testing only; the best number found goes into `DEFAULT_INTAKE_POWER`.
- **Starting values to check on the robot:** the motor direction (starts `REVERSE`, copied from the old `IntakeAlpha`), the default power, and `FLOAT` vs `BRAKE`.
- **Not in this class:** a feed-to-shooter mode or ball counting. The firing gate and any different feed power are a later session.

## 7. Session protocol (for Claude Code)

> Read `ARCHITECTURE.md` and `CLAUDE.md`. Read the reference code in section 4 and the closest SDK sample. Then do the task. Modify only the files the task lists. Propose before coding unless the task already says the design is approved. Ask before adding dependencies, changing hardware config names, or inventing new conventions. **If you hit a `[CONFIRM]` marker relevant to the task, stop and ask.**

## 8. Open decisions (`[CONFIRM]` checklist)

- [x] Package layout (section 2): a `subsystems/` folder, and subsystem class names end in `Subsystem`. (The `testing/` move below is still open.)
- [ ] Move the Alpha test OpModes into `testing/`?
- [ ] Gold-standard subsystem class (section 4)
- [ ] goBILDA encoder counts per revolution: 28 or 112? Measure on the robot (see `CLAUDE.md`).
- [ ] goBILDA motor series: all motors are believed to be 5203 (not 5202). Confirm on the motors' labels before writing `TurretSubsystem` constants.
- [ ] Turret ring gear tooth count (believed 48T; both pinions are 16T). Count it in Onshape.
- [ ] Evaluate a 360° turret (cables and slip ring, wrap-around logic). Finish the 270° version first.
- [ ] Main branch: `main` or `master`?
- [x] Vision source: **Limelight 3A** (final, no webcam or `VisionPortal`). It plugs into the Control Hub's USB 3.0 port with a USB-C to USB-A cable (one cable for power and data).

## 9. Mentor checklist (anti-drift)

When reviewing a change, check that:
- [ ] It's an iterative OpMode or a plain subsystem class, not FTCLib-style code (no base class, no commands, no scheduler)
- [ ] Subsystem class names end in `Subsystem`, and OpMode names don't
- [ ] Only the owning subsystem touches each motor or servo
- [ ] No gamepad code outside TeleOp OpModes
- [ ] Tunable numbers are named constants with explanatory comments
- [ ] Each constant lives with the class it describes (see "Where constants live"), with no duplicate copies and no plain `Constants` class
- [ ] Hardware config names match this doc
- [ ] A student could explain every line out loud
