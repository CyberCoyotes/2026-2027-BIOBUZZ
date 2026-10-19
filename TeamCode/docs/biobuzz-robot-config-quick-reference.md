# BIOBUZZ Robot Config: Quick Reference

Print this and keep it with the robot. The names below are **exactly what the code looks up**, copied from the subsystem classes. On the Driver Station, a name that is off by one letter means "device not found" and the OpMode crashes at init.

Robots: practice bot, **11940**, **22091**. All three use the same names.

Fill in the **Port** column in pen when you wire each robot. The ports are not recorded anywhere in the code.

---

## Motors (Control Hub / Expansion Hub motor ports)

| Config name | Device type | Subsystem | Direction in code | Port (write in) |
|---|---|---|---|---|
| `left_front` | goBILDA motor (mecanum drive) | DriveSubsystem | REVERSE | ____ |
| `left_rear` | goBILDA motor (mecanum drive) | DriveSubsystem | REVERSE | ____ |
| `right_front` | goBILDA motor (mecanum drive) | DriveSubsystem | FORWARD | ____ |
| `right_rear` | goBILDA motor (mecanum drive) | DriveSubsystem | FORWARD | ____ |
| `intake` | goBILDA 5203 (intake + transfer, one motor) | IntakeSubsystem | REVERSE | ____ |
| `flywheel` | goBILDA 5202/3/4, about 4 in wheel | ShooterSubsystem | FORWARD | ____ |
| `hood_wheel` | goBILDA 5203 series, about 2 in wheel | ShooterSubsystem | FORWARD | ____ |
| `turret_motor` | goBILDA 5203 series (turret rotation) | TurretSubsystem | FORWARD | ____ |

## Encoder (no motor attached)

| Config name | Device type | Subsystem | Direction in code | Port (write in) |
|---|---|---|---|---|
| `turret_encoder` | REV Through Bore Encoder, quadrature mode, on the turret ring gear. Plug into a **motor/encoder port**. Read only. | TurretSubsystem | FORWARD | ____ |

## Servo

| Config name | Device type | Subsystem | Notes | Port (write in) |
|---|---|---|---|---|
| `shooter_servo` | goBILDA dual mode 300° torque servo, configure as plain **Servo** | ShooterServoSubsystem | Set to **positional** mode with the goBILDA Servo Programmer. It is the gate: LOADING / SHOOTING. | ____ |

## Digital sensor

| Config name | Device type | Subsystem | Notes | Port (write in) |
|---|---|---|---|---|
| `turret_home_sensor` | REV magnetic limit switch, configure as **Digital Device** (NOT "REV Touch Sensor") | TurretSubsystem | Active-low: reads LOW when the magnet is near. Re-zeros the turret angle. | ____ |

## Ethernet / USB devices

| Config name | Device type | Notes |
|---|---|---|
| `limelight` | Limelight 3A, **Ethernet device** | Lives at `172.29.0.1` (NOT 192.168.1.11). Exposure 1000 / gain 8 to start. Not used by any subsystem yet. |

---

## Zero-power behavior (what the motor does at 0 power)

| Motors | Behavior | Why |
|---|---|---|
| Drive (4) | BRAKE | Robot stops when the sticks are released |
| `flywheel`, `hood_wheel` | **FLOAT** | Never BRAKE a heavy flywheel |
| `intake` | FLOAT | |
| `turret_motor` | FLOAT normally, BRAKE while holding position | |

## Quick checks before you blame the code

- [ ] Names on the Driver Station match this sheet exactly (lowercase, underscores)
- [ ] Each device is in the port you wrote down
- [ ] Turret encoder is in a motor/encoder port, with no motor attached to it
- [ ] Home switch is a **Digital Device**, not a Touch Sensor
- [ ] Servo is in positional mode (not continuous rotation)
- [ ] Limelight appears as an Ethernet device named `limelight`
- [ ] Wheels off the ground for the first direction test

## Not wired in code yet

- **goBILDA Pinpoint odometry**: no config name chosen yet
- **Pedro Pathing** drive/localizer config: not set up yet
