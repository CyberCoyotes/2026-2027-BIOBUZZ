# CLAUDE.md — Cyber Coyotes FTC (2026-2027 BIOBUZZ)

Claude Code loads this file automatically at the start of every session. It holds the rules for working in this repo. Game and robot details are in `/docs`.

## Who this code is for

- Reed City Cyber Coyotes, a middle school FTC program with two teams, **ACME Fabrications (11940)** and **ACME Innovations (22091)**, plus a practice bot.
- The programmers are **8th graders**. Write code a 13-year-old can read, trace, and change:
  - Prefer clear, verbose code to clever code. Use full command classes, not factories or lambdas-in-lambdas.
  - Explain *why* in Javadoc and comments, not just *what*.
  - Use one idea per method. Name things for what they do in the game.
- The coach reviews all work. Pushback is welcome: flag problems, risky assumptions, or a better approach instead of quietly going along.

## Session protocol (required)

1. **Read `ARCHITECTURE.md` first.** It is the source of truth for structure and conventions.
2. **Propose before coding.** Show the design (classes, states, methods, how it wires into TeleOp/Auto) and **wait for approval** before writing code unless prior authorization is already written.
3. **One subsystem or one command per session.**
4. **`[CONFIRM]` markers** in `ARCHITECTURE.md` or `/docs` mark unresolved decisions. **Stop and ask** before doing any work that depends on one.
5. When a game fact matters, check `/docs` first. If `/docs` and the Competition Manual disagree, the manual wins. Say so.

## Stack

- Java, FTC SDK 12.x, **FTCLib 2.1.1** (command-based, mirrors FRC), **Pedro Pathing 2.1.2** for autonomous. The team does not use RoadRunner.
- Hardware: REV Control Hub + Expansion Hub, Limelight 3A, goBILDA Pinpoint odometry, goBILDA 5203 motors.
- Android build flavors deploy robot-specific OpModes to the correct Control Hub:
  - Shared code: `src/main/`
  - Robot-specific OpModes: `src/practiceBot/`, `src/team11940/`, `src/team22091/`

## Architecture rules (summary; the full rules are in `ARCHITECTURE.md`)

- Actuators extend `SubsystemBase`. Sensors (Limelight, Pinpoint) are **plain helper classes** in `common/helpers/`. They are not registered with the scheduler, and the OpMode calls their `update()` each loop.
- **State vs. status:** *state* is explicitly set by commands; *status* is observed or derived from sensors.
- **Button bindings live only in TeleOp.** Don't mention buttons in command Javadocs.
- Commands declare **every** subsystem they touch with `addRequirements(...)`.

## Naming conventions

| Thing | Convention | Example |
|---|---|---|
| Packages | lowercase | `common.subsystems` |
| Classes | PascalCase | `TurretSubsystem` |
| Variables / methods | camelCase | `targetVelocity` |
| Hardware variables | location first | `leftIntakeServo` |
| Hardware config names | snake_case | `left_intake_servo` |
| Constants | ALL_CAPS | `MAX_TURRET_DEGREES` |
| Doc filenames | kebab-case | `biobuzz-game-essentials.md` |

## Known gotchas

These cost us time before, so don't repeat them:

- **FTCLib 2.1.1 API:** use `whileHeld` / `whenPressed`. Don't use `whileTrue` / `onTrue`, which are newer WPILib names. Use `getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)`. `Button.GUIDE` does not exist.
- **Drive input axes:** strafe and rotation are negated in `DefaultDriveCommand`. The motor directions are correct; the negation is an input-convention transform, so don't "fix" the motors.
- **goBILDA 5203 encoder:** 28 PPR × 4 = **112 CPR**. At 6000 RPM, max is about 11,200 ticks/sec. A 4× error here caused flywheel underperformance last season.
- **Continuous rotation servos:** 0.0 = full reverse, 0.5 = stop, 1.0 = full forward. `setDirection(REVERSE)` has caused problems.
- **Limelight 3A:** needs an Ethernet device in the robot config named `limelight`. It lives at `172.29.0.1` on the Control Hub's internal subnet, not `192.168.1.11`. Exposure 1000 / gain 8 is a good starting point, and it can run at 90 FPS.
- **FTC SDK 12.0 breaking change:** `AprilTagDetection` may be a single tag or a **cluster**. Check with `instanceof AprilTagSingleDetection` / `AprilTagClusterDetection` and cast. Last season's AprilTag code will not compile unchanged.
- **Check the cables before the code.** Motors or sensors plugged into the wrong port are the most common time-waster.

## Dev environment

- Android Studio Narwhal 3 Feature Drop or later (the SDK needs AGP 8.13.2 / Gradle 9.1). If Android Studio offers to downgrade AGP, say no.
- On Windows, deploying may need Android Studio run as administrator. The permanent fix is to set the `GRADLE_USER_HOME` environment variable.
- The TeamCode `build.gradle` may need `compileSdk 36` to override a `build.common.gradle` mismatch.

## Docs index

- `docs/biobuzz-game-essentials.md`: scoring, ranking points, and the rules that affect code and design
- `docs/biobuzz-hive-apriltag-geometry.md`: hive and cell dimensions, tag IDs and layout, camera math
- `docs/biobuzz-robot-design.md`: current robot concept (turret shooter + Limelight), build order, and open decisions

## Heads up

2026-2027 is the **last season** for this Java/FTCLib setup. In 2027-2028 FTC moves to SystemCore. Don't over-invest in framework cleverness that won't carry over; clear, teachable code matters more.
