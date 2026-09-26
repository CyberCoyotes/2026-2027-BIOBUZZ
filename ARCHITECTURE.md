# Architecture — 2026-2027 BIOBUZZ

*This is the source of truth for how team code is structured. Claude Code and students read it before starting any coding session. `[CONFIRM]` marks a decision the coach hasn't made yet: **stop and ask** before doing work that depends on one. When a decision is made, replace the marker with the answer and mention the change in the commit message.*

## 1. Strategic context

Last season (DECODE) we used FTCLib command-based code. **This season we use the standard FTC SDK style instead**, for three reasons:

1. **It's what the FTC samples use.** Students can learn from `FtcRobotController/.../external/samples/` directly, without translating between two styles.
2. **It's easier to trace.** An iterative OpMode runs top to bottom every loop, with no scheduler deciding what runs when.
3. **It carries forward.** 2027-2028 moves FTC to SystemCore with more standard OpModes. Code habits built this year transfer.

## 2. MUST: non-negotiable conventions

### Where code lives

| Path | What goes there |
|---|---|
| `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/` | FTC's reference samples. **Read only.** |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` | Shared team code: mechanism classes, test OpModes |
| `TeamCode/src/<flavor>/java/...` | Robot-specific OpModes (once flavors are set up; see section 5) |
| `TeamCode/docs/` | Game and robot reference docs |

### Suggested package layout `[CONFIRM]`

```
org.firstinspires.ftc.teamcode
├── mechanisms/     Shooter, Turret, Intake, Drivetrain, Vision …
├── testing/        Alpha*Testing and other single-mechanism test OpModes
├── pedroPathing/   Pedro Pathing constants and tuning (per Pedro's setup guide)
└── (flavor dirs)   TeleOp and Auto OpModes for each robot
```

The existing `AlphaIntakeTesting` and `AlphaShooterTesting` sit at the package root. Moving them into `testing/` is a `[CONFIRM]`.

### OpModes

- **Iterative OpModes** (`extends OpMode`), using `init()`, `init_loop()`, `start()`, `loop()`, and `stop()`.
- Start from the closest SDK sample (`BasicOpMode_Iterative`, `ConceptAprilTag`, …) and keep its section comments ("Code to run ONCE when the driver hits INIT", etc.).
- Always stop every motor in `stop()`.
- Put all tunable numbers at the top as `private static final` constants with a comment explaining how to change them. The Alpha test OpModes are the model.
- Annotate with `@TeleOp(name = "…", group = "…")` or `@Autonomous(...)`. Test OpModes use `group = "Testing"`.

### Mechanism classes (one per mechanism)

Each mechanism is a **plain Java class**: no base class and no scheduler. It follows the idea in the SDK's `ConceptExternalHardwareClass` sample, split by mechanism.

```java
public class Shooter {
    // Constants: tunable numbers, ALL_CAPS
    // Hardware: private motor/servo/sensor fields

    public Shooter(HardwareMap hardwareMap) { /* get hardware, set directions and modes */ }

    // Actions the OpMode calls: setTargetRpm(), stop(), …
    // Questions the OpMode asks: isAtSpeed(), getRpm(), …

    public void update() { /* called once per loop() if the mechanism needs it */ }
    public void addTelemetry(Telemetry telemetry) { /* shows its own data */ }
}
```

Rules:
- **A mechanism owns its hardware.** Only `Shooter` touches the flywheel motors, only `Turret` touches the turret motor, and so on. OpModes call mechanism methods; they don't reach into the motors.
- **Constructor = setup.** It gets hardware from the `HardwareMap` and sets directions, modes, and zero-power behavior.
- **`update()` runs once per loop** for any mechanism with ongoing work (velocity checks, turret aiming, timers). The OpMode calls it; nothing calls it automatically.
- **State vs. status:** *state* is what the code told the mechanism to do (`targetRpm`, an enum like `IDLE / SPINNING_UP / READY`). *Status* is what sensors report (`getRpm()`, `hasTarget()`). Keep them separate so telemetry can show both.
- **No button references inside mechanism classes.** Gamepad handling lives only in TeleOp OpModes.
- **Sensors-only devices** (Limelight, Pinpoint) get plain wrapper classes too, e.g. `Vision`, with an `update()` that reads the latest data once per loop.

### Typical TeleOp shape

```java
@TeleOp(name = "BIOBUZZ TeleOp", group = "Competition")
public class BiobuzzTeleOp extends OpMode {
    private Drivetrain drivetrain;
    private Shooter shooter;

    @Override public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    @Override public void loop() {
        // 1. Read sensors (vision.update(), odometry, …)
        // 2. Read gamepads and decide what to do
        // 3. Tell mechanisms what to do, then call their update()
        // 4. Telemetry
    }

    @Override public void stop() {
        shooter.stop();
        drivetrain.stop();
    }
}
```

Keep that four-step order in every TeleOp `loop()`.

### Autonomous

- **Pedro Pathing** for paths. Follow Pedro's example pattern: an iterative OpMode with `follower.update()` in `loop()` and a `pathState` switch that moves from step to step.
- Auto uses the **same mechanism classes** as TeleOp.

### Naming

| Thing | Convention | Example |
|---|---|---|
| Packages | lowercase | `mechanisms` |
| Classes | PascalCase | `Turret` |
| Variables / methods | camelCase | `targetVelocity` |
| Hardware variables | location first | `leftIntakeServo` |
| Hardware config names | snake_case | `left_intake_servo` |
| Constants | ALL_CAPS | `MAX_TURRET_DEGREES` |
| Doc filenames | kebab-case | `biobuzz-robot-design.md` |

### Hardware config names (robot configuration on the Driver Station)

Names already in use:

| Name | Device |
|---|---|
| `intake_motor` | Intake motor (goBILDA 5202/3/4) |
| `flywheel_motor` | Shooter flywheel (goBILDA 5202/3/4) |
| `backspin_motor` | Backspin wheel (planned) |
| `limelight` | Limelight 3A (Ethernet device) |

Add new names here when you add hardware. Names in code must match the configuration exactly.

### Dependencies

- Pedro Pathing is the only planned outside library. **Ask before adding any other dependency.**
- **Do not modify:** anything under `FtcRobotController/`, the Gradle wrapper, or root-level Gradle files, unless the task explicitly says so.

## 3. MUST NOT: anti-patterns

- No FTCLib, commands, subsystems, or schedulers.
- No lambdas, streams, or anonymous classes. Write the loop out.
- No `sleep()` or blocking waits inside `loop()`. Use `ElapsedTime` timers instead.
- No gamepad reads inside mechanism classes.
- No two classes controlling the same motor.
- No magic numbers buried in code. Make every tunable value a named constant.
- Don't use the BIOBUZZ AprilTags to locate the robot on the field. They move with the hive; use Pinpoint for position and the tags only for aiming (see `TeamCode/docs/biobuzz-hive-apriltag-geometry.md`).

## 4. Reference code

- **Style models:** `AlphaIntakeTesting.java` and `AlphaShooterTesting.java`. They show the comment style, the constants-at-the-top pattern, and iterative OpMode structure.
- **Gold-standard mechanism class:** `[CONFIRM]`. The first mechanism class written and approved becomes the template for the rest (the `Shooter` is a likely choice).
- **SDK samples most likely to be useful:** `BasicOpMode_Iterative`, `ConceptExternalHardwareClass` + `RobotHardware`, `ConceptAprilTag`, `SensorLimelight3A`, `SensorGoBildaPinpoint`, `RobotAutoDriveToAprilTagOmni`.

## 5. Setup still needed

- [ ] Port the **build flavors** (`practiceBot`, `team11940`, `team22091`) from last season's `TeamCode/build.gradle`. They're not in this repo yet.
- [ ] Add **Pedro Pathing** to the Gradle files and set up its constants/tuning.
- [ ] `[CONFIRM]` Which git branch is the main one: `main` or `master`? The repo has both, and the claude.ai Project syncs `master`.

## 6. Hardware and behavior

The robot is still being designed. See `TeamCode/docs/biobuzz-robot-design.md` for the concept (turret shooter, backspin wheel, Limelight aiming) and its open decisions. Add each mechanism's hardware and behavior to this section once its design is approved.

## 7. Session protocol (for Claude Code)

> Read `ARCHITECTURE.md` and `CLAUDE.md`. Read the reference code in section 4 and the closest SDK sample. Then do the task. Modify only the files the task lists. Propose before coding unless the task already says the design is approved. Ask before adding dependencies, changing hardware config names, or inventing new conventions. **If you hit a `[CONFIRM]` marker relevant to the task, stop and ask.**

## 8. Open decisions (`[CONFIRM]` checklist)

- [ ] Package layout (section 2)
- [ ] Move the Alpha test OpModes into `testing/`?
- [ ] Gold-standard mechanism class (section 4)
- [ ] goBILDA encoder counts per revolution: 28 or 112? Measure on the robot (see `CLAUDE.md`).
- [ ] Main branch: `main` or `master`?
- [ ] Vision source: Limelight 3A or a webcam with `VisionPortal` (see the robot design doc)

## 9. Mentor checklist (anti-drift)

When reviewing a change, check that:
- [ ] It's an iterative OpMode or a plain mechanism class, not FTCLib-style code
- [ ] Only the owning mechanism touches each motor or servo
- [ ] No gamepad code outside TeleOp OpModes
- [ ] Tunable numbers are named constants with explanatory comments
- [ ] Hardware config names match this doc
- [ ] A student could explain every line out loud
