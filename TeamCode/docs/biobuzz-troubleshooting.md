# BIOBUZZ Troubleshooting Guide

*DRAFT. Symptom, likely cause, what to check. Everything here comes from notes already in this repo (`CLAUDE.md`, `ARCHITECTURE.md`, the other docs). Status as of 2026-11-11. When something new goes wrong and we fix it, add a row. Cause marked **(untested)** is a good guess that nobody has confirmed on the robot.*

**First rule: check the cables, ports, and config names before touching code.**

## Driver Station and config

| Symptom | Likely cause | Check |
|---|---|---|
| TeleOp stops at INIT with a hardware error | A device name in the robot config does not match the code. Most often `limelight` is missing: **every** robot config must have the Limelight Ethernet device. | Compare the config to the table in `ARCHITECTURE.md` section 2. |
| Pinpoint says NOT FOUND, all numbers 0 | The config still uses last season's name `odo`. | Rename it to `pinpoint` (steps in `pinpoint-setup-guide.md`). |
| Home switch always reads the wrong way or the code cannot read it | Configured as "REV Touch Sensor" instead of **Digital Device**. | Reconfigure as a Digital Device. |
| Gamepads do nothing | Not paired. | **Start + A** (driver) and **Start + B** (operator). Logitech switch on **X**. |
| A gamepad was paired (or re-paired) while the TeleOp was running and the alliance flipped | Pairing is **Start + A** (driver) or **Start + B** (operator), so it also presses **START** in the OpMode, and START switches the alliance on **both** gamepads. | Check the alliance on the screen and press START again if it is wrong. Better: pair before pressing START on the Driver Station. |
| The operator paired gamepad 2 while the TeleOp was running and the gate opened | **B** on gamepad 2 is the gate-OPENED button (it works between volleys), and the pairing combo Start + B presses it. (Gamepad 1's buttons do not move the gate.) | Press **X** on gamepad 2 to close it (it closes gently, and the intake reverses for that moment). Pair before START. |

## Shooting

| Symptom | Likely cause | Check |
|---|---|---|
| Turret tracks the wrong hive | Alliance is wrong. TeleOp **starts BLUE**. | Screen alliance line; **Start** on either gamepad switches it. |
| Turret does not move on its own | No target (the turret holds when it sees nothing), or gamepad 2's **A** is being held (it floats the turret). | Screen says **NO TARGET**? Point the robot at the hive. Is someone holding gamepad 2 **A**? (ALWAYS-TRACK has no off switch any more.) |
| Turret hits a limit in the wrong place, or the zero is off | Power-up position was not "on the home switch, pointing straight ahead". 0 degrees is wherever the turret sat at power-up if the switch did not re-zero it. | Reset the turret by hand on the switch and power cycle. |
| Turret coasts further than expected past +/-120 degrees | Turret power is now 100%, and stopping distance grows with speed squared. | Ease the stick into a limit. (Not yet tested at 100%.) |
| Holding the operator's Right Trigger does not start feeding | The volley is waiting. The screen says what for under **Waiting for**: **flywheel not at speed**, **turret not aimed yet (the camera says N deg off, it needs 2.0 or less)**, **turret is floating** (gamepad 2 **A** is being held), **turret not settled** (no camera in DIAL mode), or **NO TARGET** (in TABLE mode the camera must see the hive right now). | Fix what it says. The **camera** decides if the turret is aimed, not the turret's own "on target" flag. If it says the camera is off by more than 2.0 degrees and the turret is not moving, the turret may be sticking (it needs more power to start moving than auto-align gives it near the target): write down the numbers on the screen and add them to issue #63. For a shot without the camera, gamepad 2 **LOGO** switches to DIAL. |
| A volley fired, but not all the balls left | The operator let go too soon, the intake kept pausing, or the very last ball had nothing behind it to push it up. There is no timer any more: the volley feeds until RT is let go. | Keep holding RT. Look at **Intake paused** on the screen (**flywheel not at speed** or **turret not aimed**) and the **Feeding for ... intake paused ... s** line. For the last ball, **hold Y** while RT is still held: the gate lifts and pushes it in. To check the intake moves balls at all, set `INTAKE_WAITS_FOR_SPEED` and `INTAKE_WAITS_FOR_AIM` false in `FiringSequence` (`FiringSequenceAlpha` has the test list, issue #63). |
| The intake stops by itself in the middle of a volley | On purpose: while feeding it pauses when the flywheel is not at speed or the **turret is not aimed** (the camera says it is off the hive by more than 2 degrees, or lost the hive in TABLE mode). | The screen's **Intake paused** line says which. Aim the turret (let the camera see the hive). A camera that flickers pauses it for about 0.15 s each time. |
| The driver's Right Trigger does nothing | That is on purpose: only the operator's Right Trigger fires, unless **SOLO MODE** is on (a one-person match). | Press **LOGO** on gamepad 1. The screen says **SOLO MODE ON**. If it never changes, look at the INIT screen's **BUTTON TEST**: "Gamepad1 buttons now" should show **LOGO** while you hold it. (LOGO is known to work on our controllers.) |
| A ball got stuck when the gate closed | A ball half way through when the gate closes (when RT is let go) can jam. (Use **Y** to lift the last ball in before letting go.) The gate closes **gently**, which should push a single ball aside, but a ball that cannot move still gets the full push. | Write down when it happens and how many balls were left, in issue #63. Raise `GENTLE_CLOSE_SECONDS` in `ShooterServoSubsystem` (gentler, slower), or keep holding RT until the balls are gone. If the gate never closes at all, check that the OpMode calls `shooterServoSubsystem.update()` every loop. |
| The drive toggle does not work, or the robot will not drive and the stick acts like the D-pad | The physical **MODE** button was pressed on a Logitech controller. It is a hardware switch that swaps the D-pad and the left stick (its LED is lit). The code cannot read it. | Press **MODE** again so the LED goes out. The drive toggle is **BACK** (on both gamepads), not MODE. The INIT screen's **BUTTON TEST** shows what each controller reports. |
| The intake runs backward for a moment after every volley, and the driver's Right Bumper does nothing for that moment | That is on purpose: while the gate walks from OPEN (or LIFTED) to CLOSED the intake reverses automatically, to push a ball out of the gate's way. It lasts 0.5 s (the same from OPEN or from LIFTED) and the screen's intake line says "gate closing: REVERSE (automatic)". It runs at the intake's power dial setting (starts at 100%). | If a ball is spat out of the robot, or the reverse is too long, lower the intake power dial, or lower `GENTLE_CLOSE_SECONDS` in `ShooterServoSubsystem` (a faster close is a shorter reverse). If it never stops, check that the gate really reaches CLOSED and that the OpMode calls `shooterServoSubsystem.update()` every loop. |
| Holding the trigger keeps the intake running, even with no balls | On purpose: there is no "one volley per press" and no timer any more. The volley feeds until RT is let go. | Let go of RT when the balls are gone. |
| "OK to shoot? YES" flickers | RPM readings jump around by more than about 15 RPM. | Smoothing (`RPM_FILTER_TIME_CONSTANT_SECONDS`), the PIDF numbers, or the speed reading. Do not loosen the tolerance without asking the coach. |
| Wheel RPM looks about 4x too low or high | Wrong ticks-per-rev. The right numbers are **28 / 103.8 / 384.5** per output revolution (hood wheel / flywheel / turret). **Not 112.** | `CLAUDE.md` "Known gotchas". Quick check: at full power every goBILDA 5203 reads about 2,800 ticks/sec. |
| Flywheel slows down when the robot stops | Zero-power behavior is `BRAKE` on a heavy flywheel. | Flywheels use `FLOAT`, never `BRAKE`. |
| Shot misses left or right consistently | Aim offset needs tuning. | `AIM_OFFSET_DEGREES` in `VisionSubsystem`. Shots landing right of the hole: more negative. Left: more positive. Change by about 0.5 degrees at a time. |
| Wrong RPM at a spot between the measured ones | The shot table is a straight line between rows and was measured at 5 spots, with the old 6000 RPM flywheel motor. | Re-measure with `ShotTableAlpha`. Every flywheel RPM must stay at **1458 or below**. |
| Table RPM looks wrong after the camera was bumped | The table is keyed on `ty`, which depends on where the camera sits. | If the camera is moved or re-tilted, measure the table again. |
| Vision shows old data | Result is older than `MAX_RESULT_AGE_MS`, so it counts as no target. | Limelight cable and the 30 second boot. |
| Camera finds no tags | Exposure and gain may be too dark under arena lights. | Starting point exposure 1000 and gain 8. Retune under the real lights, since the tags sit in the cell's shadow. |
| Limelight cannot be reached from a laptop | It is at **172.29.0.1** on the Control Hub's internal subnet, not 192.168.1.11. | Check that address first. |
| Old AprilTag code will not compile | FTC SDK 12.0 can return a cluster. | Check `instanceof AprilTagSingleDetection` / `AprilTagClusterDetection`. (Only matters for the SDK `VisionPortal` path, not the Limelight.) |

## Driving and position

| Symptom | Likely cause | Check |
|---|---|---|
| A wheel spins the wrong way | Motor direction. | `DriveAlpha`: X, A, Y, B each spin one wheel. Do it with the robot on blocks. |
| Robot tips on a hard stop | Tall robot with `BRAKE`. | Slow mode (hold driver's Left Trigger, 40%). Consider easing the stop. |
| Pinpoint heading or distance looks wrong | Pod directions and the distance multiplier. First robot test: heading sign is correct (`INVERT_HEADING` false), sideways pod is REVERSED. The distance multiplier is still being confirmed, and last season's 1.5 was probably covering a pod setting. | `PinpointAlpha`, six-step test order in its header. |
| Pinpoint drifts or starts wrong | Robot moved during INIT. | Keep the robot still until the screen says **READY**. |
| Drive keeps dropping from FIELD to ROBOT centric while driving | The Pinpoint stopped being READY for more than 0.25 s, or came back with its heading far off (it restarted). A weak or old battery that sags under load is the usual cause. Shorter blinks are forgiven now. | Read the reason in the **DRIVE MODE** line. Check the **Pinpoint status** line and the **Pinpoint glitches** count (many = voltage dips). **Read the "Last drop" line** (it names the status and how far the heading moved) and write it down: a pod `FAULT_` means a pod cable, `FAULT_BAD_READ` the I2C cable, `NOT_READY > CALIBRATING` a restart. **Change the battery** too, but a fresh battery did not stop it in the second test, so check the cables: both pod cables at the Pinpoint, and the I2C cable at the Hub (push each in, look for a cable that rubs or is pinched). Then the drive toggle (BACK on either gamepad) or the driver's D-pad Left turns field-centric back on. |
| Turret and camera stop tracking after a bump or a brownout | The Limelight restarted and came back on a different pipeline (or is still booting, about 30 s). | Check **Camera pipeline fixes** on the screen (above 0 = it restarted, and the code switched it back) and the Vision **Pipeline N (wanted 0)** line. Change the battery. Check the Limelight's USB cable. |
| Pedro path ends in the wrong place **(untested)** | Pedro's motor names or directions do not match ours, or the pod offsets are off. | `pedro-setup-and-auto-guide.md` section 7. (Pedro is set up in code but not yet run on a robot.) |

## Power

| Symptom | Likely cause | Check |
|---|---|---|
| "Battery too low to run motors and servos" from the Expansion Hub, while the Control Hub is fine | One hub's power wire or connector, not the battery. This was seen on **Expansion Hub 2** in the 10-26 robot log. | Check that hub's power wire and connector. The Driver Station's own battery warning is the only reading we have; the TeleOp no longer shows hub voltages. |
| Hub resets or motors cut out when the flywheel spins up **(untested)** | Weak battery or a loose power connector. | Swap batteries. Check the battery and hub power wires and connectors. |

## Servos

| Symptom | Likely cause | Check |
|---|---|---|
| Gate servo does not go where expected | Servo not in **positional** mode. | Set it with the goBILDA Servo Programmer, and configure it as a plain "Servo". |
| A continuous servo creeps or runs the wrong way | Continuous rotation servos: 0.0 is full reverse, **0.5 is stop**, 1.0 is full forward. `setDirection(REVERSE)` has caused problems. | Set the power numbers directly instead of using `REVERSE`. |

## Building and deploying

| Symptom | Likely cause | Check |
|---|---|---|
| Android Studio will not deploy on Windows | Needs more permission. | Run Android Studio as administrator. Permanent fix: set the `GRADLE_USER_HOME` environment variable. |
| `compileSdk` mismatch | `build.common.gradle` disagrees. | TeamCode `build.gradle` may need `compileSdk 36`. |
| Android Studio offers to downgrade AGP | Do not accept. | The SDK needs AGP 8.13.2 and Gradle 9.1. |
| Wrong robot's code deployed | Build flavors are not set up in this repo yet. | See `ARCHITECTURE.md` section 5. |

## When it is not in this table

1. Check cables and ports. 2. Check the config name. 3. Read the telemetry on the screen, both what we **told** the subsystem to do (state) and what the sensor says (status). 4. Run the matching `*Alpha` test OpMode (list in `biobuzz-opmode-controls.md`). 5. Write what you find here.
