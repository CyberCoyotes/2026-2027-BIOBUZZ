# BIOBUZZ Test and Match Log

*Add a row every time we test something, tune a number, or play a match. Never delete a row; add a new one. Dates are real dates (`YYYY-MM-DD`). This is how we find out later whether a change helped, and what to tell the judges.*

## 1. Where each tuned number lives

The code is the source of truth. This table only tells you **where to look and what to re-test when something changes.** Do not copy the current value in here (it will go out of date); look in the class.

| Number | Lives in | Re-measure when | Tested with |
|---|---|---|---|
| Hood wheel ratio (`HOOD_WHEEL_RATIO`) | `ShooterSubsystem` | The wheels, belts, or balls change | `ShooterRatioAlpha` |
| Flywheel / hood wheel max RPM | `ShooterSubsystem` | The motors change | `ShooterRPMAlpha` (full-power run) |
| At-speed tolerance, RPM smoothing, hold time | `ShooterSubsystem` | READY flickers or lags | `ShooterRPMAlpha` |
| Shot table rows (`ty` and RPM) | `ShotTable` | The camera moves, tilts, or the flywheel motor changes | `ShotTableAlpha` |
| Gate positions CLOSED / OPENED / LIFTED, travel time | `ShooterServoSubsystem` | The servo or the horn is moved | `ShooterRPMAlpha` |
| Turret ticks per degree, soft limits, zero | `TurretSubsystem` | The encoder or ring gear changes | `TurretAlpha` |
| Aim offset (`AIM_OFFSET_DEGREES`) | `VisionSubsystem` | Shots miss to one side | A fixed-spot shooting test |
| Camera pipeline index, exposure, gain | `VisionSubsystem` and the Limelight web page | Different arena lighting | `VisionAlpha` |
| Turret spin-follow gain (`TURRET_FEEDFORWARD_GAIN`) | `BiobuzzTeleOp` | The turret or its controller changes | Spin steadily while aligned and watch `tx` |
| Intake power, direction | `IntakeSubsystem` | The motor or the chute changes | `IntakeAlpha` |
| Intake reverse time after the gate closes (`CLOSE_REVERSE_SECONDS`): clears a ball out of the throat | `ShooterServoSubsystem` | The gate speed, the chute, the intake power, the battery, or the balls change | `FiringSequenceAlpha` (D-pad Left / Right nudges the time by 0.05 s, test step 11), or `BiobuzzTeleOp` (fire, let go with a ball left in the chute). Needs the slow-mo video test in section 5 |
| Drive directions, slow-mode scale | `DriveSubsystem` | Wiring changes | `DriveAlpha` |
| Pinpoint pod offsets, directions, distance multiplier | `PinpointSubsystem` | A pod moves | `PinpointAlpha` |
| Drive Forward / Drive Reverse: drive power, coast allowance (`COAST_DISTANCE_INCHES`), safety timeout, backup seconds | Each of the four `AutoBlueDriveForward24`, `AutoRedDriveForward24`, `AutoBlueDriveReverse24`, `AutoRedDriveReverse24` (and `AutoEmergencyLeave`), tuned on its own | The wheels, battery or floor change, or `DRIVE_POWER` changes (a faster robot coasts farther). Tune the coast, not `DRIVE_DISTANCE_INCHES` (the name says `24in`) | The Auto itself: run it 5 times, measure with a tape from the start spot (issue #114) |

**Per-robot warning:** the practice bot, 11940, and 22091 can differ. Say which robot every row is for. Per-robot values wait for the build flavors (`ARCHITECTURE.md` section 5).

## 2. Test log

| Date | Robot | Test OpMode | What we changed | Result (numbers, not just "good") | Next step | Who |
|---|---|---|---|---|---|---|
| | | | | | | |

### 2a. Firing sequence test (issue #122)

*Why: the gate does not always open on time and the operator presses Y or B by hand. Two robots, TABLE mode with the camera, PRE-REV off, battery 12.5 V no load (fresh off the charger it is 13.5 to 14 V). **Write your guess before you test**, then compare. How to see the numbers: `biobuzz-panels-how-to.md`.*

Test 5 volleys each, same robot, same spot: **(1)** TABLE, PRE-REV off. **(2)** TABLE, PRE-REV on (D-pad Up, wait for the wheels, then pull the trigger). **(3)** the same as (1) with a fresh battery. Copy the two `VOLLEY` lines from the log (or `Last wait` and `Last feed` in Panels) into each row.

| Date | Robot | Battery V (no load) | Test (1, 2 or 3) | Flywheel RPM at the pull | Seconds to ready | Waited on (flywheel / turret / both, s) | Y or B pressed? | Flywheel low during feed | Peak amps | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | | |

## 3. Match log

Fill in one row after **every** match, including practice matches.

| Date | Event / match | Alliance | Partner | OpMode (Auto and TeleOp) | Auto result | Tips | Park? | Score | What worked | What broke | Lowest hub voltages | Fix needed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | | | | |

## 4. Scouting notes (other teams)

For picking alliance partners later. A few words per team.

| Team | Robot type | Auto (what and where) | Can LEAVE? | Can tip? | Parks reliably? | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |

## 5. Practice time log

Quick measurements for the strategy docs. These fill in the blanks in `biobuzz-auto-strategy.md` section 6 and `biobuzz-game-strategy.md` section 4.

| Date | What we timed | Seconds | Robot | Notes |
|---|---|---|---|---|
| | Shoot 4 balls (flywheel already at speed) | | | |
| | Full scoring cycle (reload to last shot) | | | |
| | Drive from shooting spot to the LOADING ZONE | | | |
| | Balls it took to tip the hive | (count) | | |
| | Intake reverse after the gate closes: SHORTEST time that always clears the throat (10 tries in a row, with 1 ball left, then 2, then 3) | | | Phone slow-mo video of the throat, frame count from the gate command to the ball clear. Replaces the guess `CLOSE_REVERSE_SECONDS` 0.50. Chute sensors are issue #116 |
| | Intake reverse after the gate closes: LONGEST time before a ball is pushed out the FRONT of the intake | | | Same video. The final number goes between the shortest and the longest, closer to the shortest. Test on a fresh battery AND a low one |
