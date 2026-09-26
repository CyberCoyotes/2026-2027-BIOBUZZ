# CLAUDE.md — Cyber Coyotes FTC (2026-2027 BIOBUZZ)

Claude Code loads this file automatically at the start of every session. It holds the rules for working in this repo. Structure and conventions are in `ARCHITECTURE.md`; game and robot details are in `TeamCode/docs/`.

## Who this code is for

- Reed City Cyber Coyotes, a middle school FTC program with two teams, **ACME Fabrications (11940)** and **ACME Innovations (22091)**, plus a practice bot.
- The programmers are **8th graders**. Write code a 13-year-old can read, trace, and change:
  - Prefer clear, verbose code to clever code. Don't use lambdas, streams, or deep inheritance.
  - Explain *why* in comments, not just *what*. The existing `Alpha*Testing.java` OpModes show the comment style we want.
  - Use one idea per method. Name things for what they do in the game.
- The coach reviews all work. Pushback is welcome: flag problems, risky assumptions, or a better approach instead of quietly going along.

## Session protocol (required)

1. **Read `ARCHITECTURE.md` first.** It is the source of truth for structure and conventions.
2. **Propose before coding.** Show the design (classes, states, methods, how it wires into TeleOp/Auto) and **wait for approval** before writing code unless prior authorization is already written.
3. **One mechanism class or one OpMode per session.**
4. **`[CONFIRM]` markers** in `ARCHITECTURE.md` or `TeamCode/docs/` mark unresolved decisions. **Stop and ask** before doing any work that depends on one.
5. When a game fact matters, check `TeamCode/docs/` first. If the docs and the Competition Manual disagree, the manual wins. Say so.

## The approach this season: standard FTC SDK style

- **No FTCLib and no command-based code** (no subsystems, commands, or schedulers). We follow the patterns in the FTC SDK samples.
- **Reference code:** `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/`. Start from the closest sample and keep its structure and comment style. **Don't modify anything under `FtcRobotController/`.**
- **Our code:** everything under `TeamCode/src/`.
- **Iterative OpModes** (`extends OpMode`: `init`, `init_loop`, `start`, `loop`, `stop`), like `BasicOpMode_Iterative` and our `AlphaIntakeTesting`.
- **One class per mechanism** (`Shooter`, `Turret`, `Intake`, …). Each class owns its own hardware and methods, and OpModes use the mechanism classes. See `ARCHITECTURE.md`.

## Stack

- Java, FTC SDK 12.x, **Pedro Pathing** for autonomous. The team does not use RoadRunner or FTCLib.
- Hardware: REV Control Hub + Expansion Hub, Limelight 3A, goBILDA Pinpoint odometry, goBILDA 5203 motors.
- Android build flavors deploy robot-specific OpModes to the correct Control Hub (`practiceBot`, `team11940`, `team22091`). Shared code lives in `src/main/`. The flavors are not set up in this repo yet; see `ARCHITECTURE.md`.

## Naming conventions

| Thing | Convention | Example |
|---|---|---|
| Packages | lowercase | `mechanisms` |
| Classes | PascalCase | `Turret` |
| Variables / methods | camelCase | `targetVelocity` |
| Hardware variables | location first | `leftIntakeServo` |
| Hardware config names | snake_case | `left_intake_servo` |
| Constants | ALL_CAPS | `MAX_TURRET_DEGREES` |
| Doc filenames | kebab-case | `biobuzz-game-essentials.md` |

## Known gotchas

These cost us time before, so don't repeat them:

- **goBILDA 5202/3/4 encoder ticks:** `[CONFIRM]` Our notes from last season say 112 counts per motor revolution, but goBILDA's spec and the SDK's goBILDA motor type use **28 counts per motor revolution** (7 pulses × 4 edges). The Alpha test code uses 28. Before tuning any velocity control, check on the robot: at full power with no load, a 1:1 (6000 RPM) motor should read about **2,800 ticks/sec** from `getVelocity()` if 28 is right, or about 11,200 if 112 is right.
- **Continuous rotation servos:** 0.0 = full reverse, 0.5 = stop, 1.0 = full forward. `setDirection(REVERSE)` has caused problems.
- **Limelight 3A:** needs an Ethernet device in the robot config named `limelight`. It lives at `172.29.0.1` on the Control Hub's internal subnet, not `192.168.1.11`. Exposure 1000 / gain 8 is a good starting point, and it can run at 90 FPS.
- **FTC SDK 12.0 breaking change:** `AprilTagDetection` may be a single tag or a **cluster**. Check with `instanceof AprilTagSingleDetection` / `AprilTagClusterDetection` and cast. Older AprilTag code will not compile unchanged.
- **Flywheels:** use `FLOAT` zero-power behavior, never `BRAKE`, on a heavy flywheel.
- **Check the cables before the code.** Motors or sensors plugged into the wrong port are the most common time-waster.

## Dev environment

- Android Studio Narwhal 3 Feature Drop or later (the SDK needs AGP 8.13.2 / Gradle 9.1). If Android Studio offers to downgrade AGP, say no.
- On Windows, deploying may need Android Studio run as administrator. The permanent fix is to set the `GRADLE_USER_HOME` environment variable.
- The TeamCode `build.gradle` may need `compileSdk 36` to override a `build.common.gradle` mismatch.

## Docs index

- `ARCHITECTURE.md`: structure, patterns, and rules for all team code
- `TeamCode/docs/biobuzz-game-essentials.md`: scoring, ranking points, and the rules that affect code and design
- `TeamCode/docs/biobuzz-hive-apriltag-geometry.md`: hive and cell dimensions, tag IDs and layout, camera math
- `TeamCode/docs/biobuzz-robot-design.md`: current robot concept (turret shooter + Limelight), build order, and open decisions

## Heads up

2026-2027 is the **last season** of FTC Java in its current form. In 2027-2028 FTC moves to SystemCore with more standard OpModes, which is one reason we're using the standard SDK style now.
