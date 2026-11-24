# BIOBUZZ TeleOp Controls: Driver and Operator

**Print pages 1 and 2 only.** Page 1 is the **driver's** table and page 2 is the **operator's** table, each on its own page. Everything after them is the explanation, for reading before practice. This sheet matches **`TeleOp - Alpha 15`** (`BiobuzzTeleOp.java`). If the code changes, update this sheet. The controls for every other OpMode (the test Alphas) are in `biobuzz-opmode-controls.md`. **Last checked against the code: 2026-11-24 (Issue #118).** Every gamepad read in `BiobuzzTeleOp`, the dial ranges and steps, the wall shot numbers and the slow-mode speed matched both tables. The Autos you can pick before this TeleOp are on `biobuzz-driver-station-autos.md`.

**How to read the tables:** every button has a row, except the two thumbstick presses. A **—** means **nothing is assigned**, so it is free if we need a new control. **HOLD** means the action lasts only while the button is held. The other buttons are **one move per press**: holding does not repeat. Words in *italics* point to a section in the notes after the tables.

## Gamepad 1: DRIVER

**Your job:** drive and run the **intake**. The **operator fires**. The turret aims itself. **Before START:** pair with **Start + A**, then check the **alliance** (Start) on the screen.

| Button | What it does |
|---|---|
| **Left Stick** | Drive forward / backward and slide (strafe) left / right |
| **Right Stick** | Turn (push right to turn right) |
| **A** | — |
| **B** (HOLD) | **SOLO MODE only:** FORCED SHOT (same as the operator's B). Does nothing otherwise |
| **X** | — |
| **Y** (HOLD) | **SOLO MODE only:** lift the shooter gate (same as the operator's Y). Does nothing otherwise |
| **Left Bumper (LB)** | **REVERSE** the intake (push balls out). Beats everything, so it always clears a jam |
| **Right Bumper (RB)** | **INTAKE** (pull balls in). **The intake is the driver's only** |
| **Left Trigger (LT)** (HOLD) | **SLOW MODE** (40% speed) for careful lining up |
| **Right Trigger (RT)** (HOLD) | **Does not fire.** **SOLO MODE only:** fires a volley (same as the operator's RT) |
| **D-pad Up** | **PRE-REV** on / off: the wheels spin without firing. Starts **OFF**. Screen says **PRE-REV ON** |
| **D-pad Down** | — |
| **D-pad Left** | **RE-AIM:** "the way the robot faces right now is **away from me**." Press it if the robot was turned or bumped |
| **D-pad Right** | — |
| **Start** | **ALLIANCE** BLUE / RED (same as the operator's). Starts **BLUE**. **Check the screen** |
| **Back** | Drive **FIELD-CENTRIC / ROBOT-CENTRIC** (same as the operator's). Starts **FIELD-CENTRIC** |
| **Guide** (center logo) | **SOLO MODE** on / off, for a one-person match. Starts **OFF**. Screen says **SOLO MODE ON** |

**Never press the physical MODE button** while driving: it swaps the D-pad and left stick (its LED lights), and the robot will not drive. If the LED is on, press MODE again. **Jam in the shooter wheels:** power off and clear it by hand. They cannot be reversed from the controllers.

<div style="page-break-after: always"></div>

## Gamepad 2: OPERATOR

**Your job:** **fire** (RT), set the shooter speeds, move the shooter gate, force a shot when the checks are wrong (B), move the **flower picker** (the bumpers), and check the alliance. **The intake is the driver's.** You do not aim the turret: it tracks the hive by itself. **Every button always does the same thing** (no layers). **Before START:** pair with **Start + B**, then check the **alliance** on the screen.

| Button | What it does |
|---|---|
| **Left Stick** | — |
| **Right Stick** | — |
| **A** (HOLD) | **FLOAT the turret** so you can turn it by hand. Let go and it tracks the hive again |
| **B** (HOLD) | **FORCED SHOT:** wheels on, gate **OPENED**, intake running **right now**, even if the READY checks say "not ready". Let go and the gate closes. Use it when the camera or the READY check is wrong |
| **X** | Gate **CLOSED**. **Between volleys only** |
| **Y** (HOLD) | Gate **LIFTED**, to push the very last ball into the flywheel. Let go: back to OPENED while feeding, CLOSED otherwise |
| **Left Bumper (LB)** | **FLOWER PICKER DOWN** (one move per press) |
| **Right Bumper (RB)** | **FLOWER PICKER UP** (one move per press). If both bumpers are pressed at once, **UP wins** |
| **Left Trigger (LT)** (HOLD) | **WALL SHOT:** dumb shot from up against the wall. Turret straight ahead, fixed wheel speeds (1300 / 2600 RPM), no camera. Gate opens after 1.5 s. Let go and the gate closes |
| **Right Trigger (RT)** (HOLD) | **FIRE a volley.** Waits until ready, then opens the gate and **keeps feeding while you hold**. Let go to end it. No one-volley limit |
| **D-pad Up** | Flywheel target **UP** 100 RPM (range 500 to 1450) |
| **D-pad Down** | Flywheel target **DOWN** 100 RPM |
| **D-pad Left** | Ratio (hood wheel / flywheel) **DOWN** 0.05 (range 1.00 to 3.00) |
| **D-pad Right** | Ratio (hood wheel / flywheel) **UP** 0.05 |
| **Start** | **ALLIANCE** BLUE / RED (same as the driver's). Starts **BLUE**. **Check the screen**: the wrong alliance aims at the other hive |
| **Back** | Drive **FIELD-CENTRIC / ROBOT-CENTRIC** (same as the driver's) |
| **Guide** (center **LOGO**) | Flywheel speed source **TABLE / DIAL**. Starts **TABLE** (the camera) |

**Starts at:** flywheel **1300 RPM**, ratio **2.00** (hood wheel 2600 RPM). **X and Y at once:** CLOSED wins. **During a volley or forced shot** X does nothing. **While LT (wall shot) is held,** B and RT do nothing. **Pairing while the TeleOp runs** flips the alliance and starts a forced shot: pair before START.

<div style="page-break-after: always"></div>

# Notes (not part of the two-page print)

## Setup and safety

**Pairing:** on the Driver Station, press **Start + A** on the driver's controller (gamepad 1) and **Start + B** on the operator's controller (gamepad 2).

**Pairing warning:** if you pair or re-pair either controller while this OpMode is running, **Start flips the alliance** (it may flip to the wrong one), and **B starts a FORCED SHOT** (on gamepad 2: the wheels spin and the gate opens and the intake runs, for as long as the pairing combo holds B). **Keep hands and balls clear.** Pair before START, or check the **Alliance** line and the gate on the screen afterward.

**Logitech controller (F310 / F710):** set the switch on the back to **X** (XInput mode), not D. The buttons are then labeled A, B, X, Y like the code. Bumpers are **LB / RB** and triggers are **LT / RT**. (The screen may call it an "XBOX_360" controller: that is only the SDK's label for a Logitech in X mode.)

**Do not press the physical MODE button while driving.** On a Logitech controller MODE is a hardware switch that swaps the D-pad and the left stick (its LED lights). With it on, the drive stick would read as the D-pad and the robot would not drive. If the LED is on, press MODE again. The robot's code cannot read the MODE button, so the drive toggle is on **BACK** (on both controllers) instead.

**Why BACK, and not the physical MODE button?** The code cannot read the Logitech MODE button: the FTC SDK's `Gamepad` class has no field for it, and Logitech says MODE is a hardware switch that swaps the D-pad and the left stick. The INIT screen has a **BUTTON TEST** that lists every button each gamepad reports, if you want to see for yourself. If a button there ever changes when you press MODE, tell us which one and the toggle can move onto it (one line each in `BiobuzzTeleOp`: `isDriveToggleButtonPressedOnDriverPad()` and `isDriveToggleButtonPressedOnOperatorPad()`).

**Safety:** keep hands, hair, and loose clothing away from the shooter wheels, and keep everyone out of the path of the ball. Keep a hand near the **Stop** button when testing the turret.

**Free buttons right now (blank in the tables):** on gamepad 1: A, X, D-pad Down, D-pad Right. On gamepad 2: both sticks. (Gamepad 1's Right Trigger, Y and B only do something in SOLO MODE, its D-pad Up is PRE-REV, Start is the alliance, Back is the drive toggle and LOGO is the SOLO MODE toggle.)

## The firing sequence (the operator holds the Right Trigger)

The **operator fires** (gamepad 2 Right Trigger): the robot waits until it is ready, then opens the gate and feeds the balls by itself. **The driver can fire only in SOLO MODE** (a one-person match), switched with the **Guide** (LOGO) button.

1. **The operator holds RT** (the driver's RT in SOLO MODE). The wheels spin (if they are not already) and the screen says **Waiting for** whatever is missing, with numbers: **flywheel not at speed**, **turret not aimed yet (the camera says 2.6 deg off, it needs 2.0 or less)**, **turret is floating** (the operator is holding A), or **NO TARGET** (in TABLE mode the camera must see the hive). The **camera** decides whether the turret is aimed: it must read within 2 degrees for a moment (0.15 s), so the volley does not start while the turret is still swinging in.
2. **When everything is ready** the gate **OPENS and STAYS OPEN**, and the intake feeds balls for as long as the operator **keeps holding RT** (there is **no timer**). The intake **pauses by itself** when the flywheel is not at speed or when the **turret is not aimed** (the camera says it is off the hive by more than 2 degrees, or lost the hive in TABLE mode). The screen says which under **Intake paused**, and the intake goes on when it is aimed again.
3. **Let go of RT to end the volley.** The gate **CLOSES** and the intake stops. (There is **no automatic lift step**: its timing was off and it wedged the last ball. The operator lifts by hand with **Y**, below.) The screen shows **Sequence step**, **Feeding for** while it feeds, and the log of the last volley (seconds to ready, total seconds, flywheel dips if the balls are heavy enough to cause any).
4. **There is no "one volley per press" limit.** Keep holding and the intake keeps feeding whatever comes in (3 or 4 balls is normal). Let go when the balls are gone. **Letting go at any time**, even while it is still waiting, closes the gate and stops the intake; balls not yet shot stay in the chute.

- **The gate closes at the servo's top speed.** Every gate move is instant. (We tried a slow, gentle close and dropped it: the servo cannot be made slower than it is without a long walk, and we wanted a fast close.) A single ball left in the way is pushed out by the **intake reversing** for a moment (next point). Pressing STOP closes it at once.
- **The intake REVERSES by itself after the gate closes.** Any time the gate closes from OPEN (or from LIFTED), the intake runs backward for **0.5 s** (the same from both), then stops (or goes back to whatever the driver is holding). It is a **timer, not a measurement** (a servo cannot say when the gate is shut): the number is `CLOSE_REVERSE_SECONDS` in `ShooterServoSubsystem`, so play with it on the robot. It starts the same loop the close is commanded. It happens after every volley, after an abort, and after the operator's **X** button. It does **not** happen when the gate is already closed, and it does not happen while the gate OPENS. It runs at the intake's power dial setting (starts at 100%). **While it is happening the driver's Right Bumper (intake) does nothing**; the driver's Left Bumper (reverse) is the same thing. Pressing STOP closes the gate instantly, so there is no automatic reverse then.
- **PRE-REV (the driver's D-pad Up)** spins the wheels while you drive to the spot. When RT is pressed the volley starts as soon as the turret is ready.
- **No camera and TABLE mode:** the volley will not start. The operator switches to **DIAL** (gamepad 2 **LOGO**) for a shot without the camera.
- **Clearing a jam:** the driver holds **Left Bumper** (reverse the intake). It beats the volley. When the driver lets go, the volley's feeding carries on by itself (there is no clock to run out).
- **The flywheel RPM and the ratio stay frozen for as long as you hold RT**, so if you move to a different spot, let go and press RT again.

### The forced shot (the operator's B)
An override for when the READY checks are wrong, for example the camera is dead or the turret says "not aimed" when it is. **HOLD B**: the wheels spin, the gate **OPENS** and the intake **runs forward right now**, without waiting for the flywheel to be at speed or the turret to be aimed. It works with RT held or not. **Let go and the gate CLOSES by itself**, with the automatic reverse (0.5 s). If RT is still held then, the volley starts over and waits for READY as normal. The screen says **FORCED SHOT**. If the flywheel is slow, the shot will be weak. The driver can force a shot with **B** too, but **only in SOLO MODE** (issue #80).

### The gate buttons (the operator's X and Y)
**X (CLOSED) is an override** and only works **between volleys**. **Y is HOLD-to-LIFT**: the gate is LIFTED while Y is held, to push the very last ball up into the flywheel (with no ball behind it, nothing else would). OPENED is the position between LIFTED and CLOSED. **If X and Y are pressed at the same time, CLOSED wins.**
- **While the volley is FEEDING** (RT held, the wheels at speed): hold **Y** to lift, let go and the gate goes **back to OPENED** and the volley carries on. This is the normal way to get the last ball out. Nothing closes the gate, and there is no reverse (OPENED is not a close).
- **Between volleys:** hold **Y** to lift, let go and the gate **CLOSES**, with the automatic reverse (0.5 s).
- **While the volley is only WAITING** (RT held, but the flywheel is not at speed or the turret is not aimed yet): hold **Y** to lift anyway (issue #92). It is an override, so a volley that is stuck waiting no longer blocks you. Let go and the gate **CLOSES** again, with the automatic reverse (0.5 s), and the volley keeps waiting. The volley does **not** feed by itself while it waits: the intake is still the driver's (Right Bumper), so the driver feeds while Y is held. If the flywheel is slow, the ball goes out weakly. If you let go of RT while still holding Y, the gate closes once: press Y again to lift. (Holding B skips the wait and runs the intake for you: see the forced shot above. Y lifts during a forced shot too.)
- **In SOLO MODE the driver's Y does the same job as the operator's Y**, with all the rules above. With SOLO MODE off, the driver's Y does nothing.

### The wall shot (the operator's Left Trigger, issue #9)
A dumb shot from a known spot, up against the wall. The **turret goes straight ahead (0)** and the wheels run at **fixed speeds** (flywheel 1300, hood wheel 2600 RPM), with no camera and no READY checks. After a **1.5 s delay** the gate **OPENS** and the intake runs. **Let go and the gate CLOSES** (with the 0.5 s intake reverse) and the wheels go to 0 power (unless PRE-REV is on). While LT is held, B and RT do nothing. A still floats the turret. The driver has no copy of it (the driver's LT is SLOW MODE). The two speeds and the delay are still to be verified on the robot.

### SOLO MODE (one person)
Press **LOGO** on gamepad 1. Now the driver's Right Trigger fires too, the driver's **Y** (hold) lifts the gate, and the driver's **B** (hold) is the forced shot (the operator's buttons still work if there is one). In a solo match **nobody can use the rest of the operator's buttons**: gate CLOSED (X), the wall shot (LT), the dials, DIAL / TABLE (gamepad 2 LOGO) and the flower picker (gamepad 2 bumpers). You **can** still set the alliance (**START**) and switch field / robot centric (**BACK**) yourself. The speed source now starts on **TABLE**, so in a solo match **the camera must see the hive before a volley will start**. If the camera cannot see it, someone has to press gamepad 2 **LOGO** to switch to **DIAL** before the match. LOGO is known to reach the Robot Controller on our controllers; the INIT screen's **BUTTON TEST** shows every button each gamepad reports if you want to check.

### Not tested on a robot yet
Whether the last ball leaves with the Y lift, whether pausing the intake when the turret is off feels right (a flickering camera pauses it for a moment), and whether a ball gets caught when the gate closes.

## Driver tips
- The robot starts **FIELD-CENTRIC**: pushing the left stick forward moves the robot **away from you**, even after the robot turns. The screen shows **DRIVE MODE** right under the alliance, and says which wall "forward" points at (**BLUE driver station: toward the LEFT wall**, **RED: toward the RIGHT wall**). **BACK** (yours) or the operator's **BACK** switches to **ROBOT-CENTRIC**, where forward is the way the robot is facing.
- Field-centric assumes the **blue drivers stand on the right and the red drivers on the left**, and that the robot starts TeleOp **facing the way it started Auto** (blue down the field, red up the field). If the robot was turned or bumped, or you start a practice run another way, press **D-pad Left** (re-aim). It only changes which way "forward" means: it does not reset the gyro or the Pinpoint.
- **Where the drive's heading comes from:** the **Control Hub's built-in gyro (IMU)**, not the Pinpoint. The gyro is inside the hub with no cable to wiggle, so a Pinpoint blink or restart **cannot** drop the drive out of field-centric (it used to, and then the robot suddenly drove robot-centric). The drive only goes ROBOT-CENTRIC when someone presses **BACK**, or if the config has no IMU (the screen says so under **DRIVE MODE**). A Pinpoint blink now only costs the **turret** its hive memory, and the screen still counts them under **Pinpoint glitches**. Lots of glitches or **POWER** dips means **change the battery** and check the cables. Do **not** change the alliance after START to fix a drive direction: the alliance only changes the camera's hive then. Use **D-pad Left**.
- **Keep the robot still during INIT.** The Pinpoint odometry calibrates its gyro then. Before START, check that the screen says **Pinpoint status: READY**.
- **The turret keeps pointing at the hive while you spin, even if the camera loses it.** The Pinpoint remembers which way the hive was. It gives up (screen: **Hive memory ... STALE**, turret holds still) if you drive more than **12 inches** or **5 seconds** pass without the camera seeing the hive. When the camera sees the hive again it takes over.
- The turret **always tracks the hive**, so you don't need to hold anything to aim. If the screen says **NO TARGET**, the turret just holds where it is, so check the alliance and point the robot toward the hive.
- The operator's **A** (hold) floats the turret, so don't be surprised if the turret stops following while they hold it.
- **The shooter gate is the sequence's job during a volley**, and the operator's (X / Y between volleys, B for a forced shot, LT for the wall shot, all on gamepad 2). Your X does nothing, and your Y (lift) and B (forced shot) only work in SOLO MODE.
- You do not wait for "OK to shoot" any more: the volley waits for you. It checks the **flywheel only** (within 25 RPM for 0.15 s), plus the turret. The screen also shows the **hood wheel's error** and the old **"OK to shoot?"** line (both wheels), for information.
- **There is no way to reverse the shooter wheels** from the controllers. If a ball jams in the wheels, stop the robot and clear it by hand (power off first). The intake REVERSE (your Left Bumper) still works for the intake.

## The turret: ALWAYS-TRACK
The turret follows the hive **all the time**, with no button held, and it also follows the robot's spin. There is **no switch** for it any more: it is always ON. It never moves during INIT.
- **A (hold)** floats the turret so you can turn it by hand (for example, to line it up before a match). Let go and it goes back to tracking.
- If the camera loses the hive, the turret **holds** where it is. It does not search.
- **Check the alliance (Start button, on either gamepad) before START.** The wrong alliance tracks the other team's hive.
- The turret will not go past **±120 degrees** (see `TurretSubsystem`). **0 degrees** is where the turret pointed when the Control Hub powered on, because the home switch is not installed yet. Power on with the turret pointing straight ahead, or the limits will be measured from the wrong place.

## Alliance and speed source
**DIAL** is the speed the operator dialed in (below), and needs no camera. **TABLE** uses the shot table: while the camera sees the hive, the flywheel speed comes from how high the hive's tags appear (closer = lower RPM). If the camera loses the hive, the table's last speed is kept for **1 second**, then the flywheel goes back to the dial. The screen says which one is in use. The hood wheel is always the flywheel speed x the ratio dial.

## Shooter speeds (the dials)
These set the RPMs the **Right Trigger** shot uses. The wheels only spin while the trigger is held (or PRE-REV is on), and the shooter's speed control holds each wheel at its target RPM. **During a volley the dials are frozen** (the flywheel RPM and the ratio keep their values until the volley ends), so turning a dial mid-volley does nothing until it is over.

Starting dials: flywheel **1300 RPM** (lowest 500, highest 1450), ratio **2.00** (lowest 1.00, highest 3.00), so the hood wheel starts at **2600 RPM**. 1300 is the speed that looked about right on the robot. It is also the speed used in TABLE mode when the camera has not seen the hive lately. In TABLE mode the dial does not set the flywheel speed, but the ratio dial still sets the hood wheel.

Hood wheel target RPM = flywheel RPM x ratio. The flywheel is the **1620 RPM** motor, so the shooter will not take it past **1458 RPM** (`FLYWHEEL_MAX_TARGET_RPM` in `ShooterSubsystem`), and the dial stops at 1450. The hood wheel is a 6000 RPM motor with a limit of 5400. If either wheel would go past its limit, the screen says **CAPPED** and the real ratio is lower than the dial says. At ratio 2.00 the flywheel is always the one that runs out first.

The shooter wheels **cannot be reversed** from the controllers any more (the old operator bumper jam-clearing was removed in issue #74).

## Operator tips
- The intake is the **driver's only**. Your bumpers move the **flower picker**: **Right Bumper = UP**, **Left Bumper = DOWN** (issue #75). The coach chose the operator, so the driver can drive and run the intake at the same time. At **INIT** the bar moves to UP, so keep hands and cables clear of it. The four servo positions are in `FlowerPickerSubsystem`. In SOLO MODE nobody can move the picker.
- The gate positions are tuned in `ShooterServoSubsystem` (`CLOSED_POSITION`, `OPENED_POSITION`, `LIFTED_POSITION`). The screen shows the position we **commanded**: a servo cannot report where it really is.
- **Gate priority:** if X and Y are pressed at the same time, **CLOSED wins**. While a volley or a forced shot is running, X does nothing: the firing sequence runs the gate (Y still lifts it while feeding, see above).
