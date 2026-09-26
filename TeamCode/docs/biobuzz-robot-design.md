# BIOBUZZ Robot Design (2026-2027)

*The current robot concept and the decisions behind it. Update this file when a decision changes, and mark anything still open with `[CONFIRM]`.*

## Concept

- **Drivetrain:** reusing both mecanum chassis from last season. Pinpoint odometry, Pedro Pathing for auto.
- **Shooter:** **weighted flywheel** plus an **independently controlled backspin wheel** at the top of the release arc.
- **Turret:** the shooter sits on a **rotating turret** aimed with **Limelight 3A + AprilTags**. Vision sets both turret angle and flywheel speed.
- **Game pieces:** the shooter is designed for **POLLEN only** for now (2.8 in balls).

## Why

- Hive tips are worth 20 points and drive two of the ranking points. Everything else is worth 1–5. See `biobuzz-game-essentials.md`.
- The raised cell's opening is **53.5–65.6 in high**, so the ball has to climb about 40 in from a shooter exit around 15–18 in.
- The shooting side flips after every tip. A turret lets us shoot from anywhere on the correct side without precise chassis aiming.

## Mechanism plan (proposal; follows `ARCHITECTURE.md`)

These are plain mechanism classes used by iterative OpModes. This season has no FTCLib and no subsystems.

| Class | Job |
|---|---|
| `Shooter` | Flywheel + backspin velocity control; answers `isAtSpeed()` |
| `Turret` | Turret angle control, soft limits, homing |
| `Vision` | Limelight wrapper: `tx`, `ty`, visible tag IDs, alliance filtering |
| `Intake` / feeder | Collect and feed balls; max 4 held |
| `Drivetrain` | Mecanum drive for TeleOp; Pedro Pathing handles Auto |

The specific classes, states, and methods are decided one session at a time, proposal first.

## Control approach

- **Aim:** turn the turret until the target cluster's `tx` ≈ 0. Filter tags to our alliance, and target the **higher** cluster. See the geometry doc.
- **Distance:** `d = (50 − lensHeight) / tan(cameraTilt + ty)`.
- **Flywheel speed:** a **lookup table** of distance → flywheel RPM (and backspin RPM), measured at 4–6 distances and interpolated between them. Don't use physics equations. Students can re-tune a table at an event.
- **Firing gate:** only feed a ball when the flywheel (and backspin wheel) are within a velocity tolerance **and** the turret is on target.
- **Velocity control:** `DcMotorEx.setVelocity()` with tuned PIDF. Confirm the encoder counts per revolution first (28 or 112, see `CLAUDE.md`).
- **Backspin:** start at a **fixed ratio** to the flywheel speed. Tune it separately only if testing shows spin matters.
- **Target entry angle:** a ball coming down at ~30° arrives square-on to the tilted window and has the largest usable opening.

## Fallback mode (required)

Build this first, and keep it working all season:

- The turret locks straight ahead at a known angle.
- There are **2–3 preset shooting spots**, each with fixed flywheel and backspin speeds, selected by the driver.
- There is no vision dependency. If the Limelight fails or the vision code isn't ready, the robot can still tip the hive.

## Limelight 3A mounting

- It goes **on the turret**, facing the same way as the shooter, **as low as practical**.
- **Tilt it up about 40°** as a starting point. Recalculate once the lens height is known (see the geometry doc).
- Plan the cable routing and turret rotation limits so the Ethernet and power cables can't wrap or snag.

## Open decisions `[CONFIRM]`

- [ ] **Vision source:** Limelight 3A, or a webcam with the SDK `VisionPortal`? SDK 12.0 clusters put the origin at the cell opening, which makes aiming simpler. The Limelight is faster and already in our code. Decide before writing turret aiming code.
- [ ] Turret range of motion and homing method (limit switch versus absolute encoder)
- [ ] Turret drive: motor + encoder, or servo
- [ ] Lens height and final camera tilt
- [ ] Number of balls needed to tip the hive
- [ ] Whether nectar (3.6 in) is ever added to the shooter
