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
| Drive directions, slow-mode scale | `DriveSubsystem` | Wiring changes | `DriveAlpha` |
| Pinpoint pod offsets, directions, distance multiplier | `PinpointSubsystem` | A pod moves | `PinpointAlpha` |

**Per-robot warning:** the practice bot, 11940, and 22091 can differ. Say which robot every row is for. Per-robot values wait for the build flavors (`ARCHITECTURE.md` section 5).

## 2. Test log

| Date | Robot | Test OpMode | What we changed | Result (numbers, not just "good") | Next step | Who |
|---|---|---|---|---|---|---|
| | | | | | | |

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
