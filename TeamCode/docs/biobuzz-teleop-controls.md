# BIOBUZZ TeleOp Controls: Driver and Operator

Print this and keep it at the driver station. It matches **`TeleOp - Alpha 13`** (`BiobuzzTeleOp.java`). If the code changes, update this sheet. The controls for every other OpMode (the test Alphas) are in `biobuzz-opmode-controls.md`.

**Pairing:** on the Driver Station, press **Start + A** on the driver's controller (gamepad 1) and **Start + B** on the operator's controller (gamepad 2).
**Logitech controller (F310 / F710):** set the switch on the back to **X** (XInput mode), not D. The buttons are then labeled A, B, X, Y like the code. Bumpers are **LB / RB** and triggers are **LT / RT**. Left Trigger = LT, Right Trigger = RT. (The screen may call it an "XBOX_360" controller: that is only the SDK's label for a Logitech in X mode.)
**Do not press the physical MODE button while driving.** On a Logitech controller MODE is a hardware switch that swaps the D-pad and the left stick (its LED lights). With it on, the drive stick would read as the D-pad and the robot would not drive. If the LED is on, press MODE again. The robot's code cannot read the MODE button, so the drive toggle is on **BACK** (on both controllers) instead.

**Safety:** keep hands, hair, and loose clothing away from the shooter wheels, and keep everyone out of the path of the ball. Keep a hand near the **Stop** button when testing the turret.

**How to read the tables:** every button on the controller has a row, except the two thumbstick presses. A blank row (**—**) means **nothing is assigned to it**, so it is free if we need a new control. (If a thumbstick press is used, it is named in the notes under the table.)

## Gamepad 1: DRIVER

The driver drives and runs the **intake** (the intake is the driver's only). **The operator fires** (gamepad 2 Right Trigger): the robot waits until it is ready, then opens the gate and feeds the balls by itself. **The driver can fire only in SOLO MODE** (a one-person match), switched with the **Guide** (LOGO) button. **The turret aims itself** at the hive all the time (see **ALWAYS-TRACK** on the operator page), so the driver does not need to aim.

| Button | What it does |
|---|---|
| **Left Stick** (move) | Drive forward / backward and slide (strafe) left / right |
| **Right Stick** (move) | Turn (push right to turn right) |
| **A** | — |
| **B** | — |
| **X** | — |
| **Y** (HOLD) | **Only in SOLO MODE** (see the Guide button): shooter gate **LIFTED**, to push the very last ball into the flywheel, the same as the operator's Y. With SOLO MODE off, it does nothing |
| **Left Bumper (LB)** | **REVERSE** the intake: push balls out. Beats the Right Bumper and a volley's feed, so clearing a jam always works |
| **Right Bumper (RB)** | **INTAKE**: pull balls in (also carries them up the transfer chute). **The intake is the driver's only**: the operator's bumpers do nothing |
| **Left Trigger (LT)** (hold) | **SLOW MODE** (40% speed) for careful lining up |
| **Right Trigger (RT)** (hold) | **Does not fire** in a normal match: the **operator's** Right Trigger fires. **Only in SOLO MODE** (see the Guide button) does your RT fire a volley, the same as the operator's |
| **D-pad Up** | **PRE-REV** on / off: the wheels spin without firing, so the spin-up is not paid at every volley. Starts **OFF**. The screen says **PRE-REV ON** |
| **D-pad Down** | — |
| **D-pad Left** | **RE-AIM** (for field-centric drive): "the way the robot faces right now is **away from me**." Press it if the robot was turned or bumped, or if a practice start was not in the Auto orientation. It does not touch the Pinpoint. It also turns field-centric back on after the safety rule (see the tips) |
| **D-pad Right** | — |
| **Start** | **ALLIANCE** BLUE / RED, the same as the operator's Start. Starts as **BLUE**. **Check it on the screen.** (**Start + A** also pairs this controller on the Driver Station: pair before START, or check the alliance afterward) |
| **Back** | Drive **FIELD-CENTRIC / ROBOT-CENTRIC** toggle, the same as the operator's Back. If the safety rule switched field-centric off, it turns it back on |
| **Guide** (center logo button) | **SOLO MODE** on / off, for a **one-person match** (starts **OFF**). While it is ON, your Right Trigger fires a volley, your **Y** lifts the gate, and the screen says **SOLO MODE ON**. Press it again to switch it OFF. It works during INIT too |

### The firing sequence (the operator holds the Right Trigger)
1. **The operator holds RT** (the driver's RT in SOLO MODE). The wheels spin (if they are not already) and the screen says **Waiting for** whatever is missing, with numbers: **flywheel not at speed**, **turret not aimed yet (the camera says 2.6 deg off, it needs 2.0 or less)**, **turret is floating** (the operator is holding A), or **NO TARGET** (in TABLE mode the camera must see the hive). The **camera** decides whether the turret is aimed: it must read within 2 degrees for a moment (0.15 s), so the volley does not start while the turret is still swinging in.
2. **When everything is ready** the gate **OPENS and STAYS OPEN**, and the intake feeds balls for as long as the operator **keeps holding RT** (there is **no timer**). The intake **pauses by itself** when the flywheel is not at speed or when the **turret is not aimed** (the camera says it is off the hive by more than 2 degrees, or lost the hive in TABLE mode). The screen says which under **Intake paused**, and the intake goes on when it is aimed again.
3. **Let go of RT to end the volley.** The gate **CLOSES** (gently) and the intake stops. (There is **no automatic lift step**: its timing was off and it wedged the last ball. The operator lifts by hand with **Y**, below.) The screen shows **Sequence step**, **Feeding for** while it feeds, and the log of the last volley (seconds to ready, total seconds, flywheel dips if the balls are heavy enough to cause any).
4. **There is no "one volley per press" limit.** Keep holding and the intake keeps feeding whatever comes in (3 or 4 balls is normal). Let go when the balls are gone. **Letting go at any time**, even while it is still waiting, closes the gate and stops the intake; balls not yet shot stay in the chute.
- **The gate closes GENTLY.** It walks toward CLOSED in about three quarters of a second instead of snapping shut, so a single ball left in the way is **pushed aside** instead of slammed or wedged. Opening is instant. (The speed is `GENTLE_CLOSE_SPEED_POSITION_PER_SECOND` in `ShooterServoSubsystem`: smaller is gentler but slower. Not tuned on a robot yet.) Pressing STOP closes it at once.
- **The intake REVERSES by itself while the gate closes.** Any time the gate walks from OPEN (or from LIFTED) to CLOSED, the intake runs backward for that whole walk (about **0.75 s** from OPEN, about **2 s** from LIFTED), then stops (or goes back to whatever the driver is holding). It starts the same loop the close starts and ends when the gate is shut. It happens after every volley, after an abort, and after the operator's **X** button. It does **not** happen when the gate is already closed, and it does not happen while the gate OPENS. It runs at the intake's power dial setting (starts at 100%). **While it is happening the driver's Right Bumper (intake) does nothing**; the driver's Left Bumper (reverse) is the same thing. Pressing STOP closes the gate instantly, so there is no automatic reverse then.
- **PRE-REV (the driver's D-pad Up)** spins the wheels while you drive to the spot. When RT is pressed the volley starts as soon as the turret is ready.
- **No camera and TABLE mode:** the volley will not start. The operator switches to **DIAL** (gamepad 2 **LOGO**) for a shot without the camera.
- **Clearing a jam:** the driver holds **Left Bumper** (reverse the intake). It beats the volley. When the driver lets go, the volley's feeding carries on by itself (there is no clock to run out).
- **The operator's gate buttons are an override.** **X** (CLOSED) and **B** (OPENED) only work **between volleys**. **Y is HOLD-to-LIFT**: the gate is LIFTED while Y is held, to push the very last ball up into the flywheel (with no ball behind it, nothing else would). OPENED is the position between LIFTED and CLOSED.
  - **While the volley is FEEDING** (RT held, the wheels at speed): hold **Y** to lift, let go and the gate goes **back to OPENED** and the volley carries on. This is the normal way to get the last ball out. Nothing closes the gate, and there is no reverse (OPENED is not a close).
  - **Between volleys:** hold **Y** to lift, let go and the gate **CLOSES gently**, with the automatic reverse (about 2 s from LIFTED).
  - **While the volley is only WAITING:** Y does nothing (the gate is held CLOSED).
  - **In SOLO MODE the driver's Y does the same job as the operator's Y**, with all three rules above. With SOLO MODE off, the driver's Y does nothing.
- **SOLO MODE (one person):** press **LOGO** on gamepad 1. Now the driver's Right Trigger fires too and the driver's **Y** (hold) lifts the gate (the operator's buttons still work if there is one). In a solo match **nobody can use the rest of the operator's buttons**: gate CLOSED / OPENED (X / B), the dials and DIAL / TABLE (gamepad 2 LOGO). You **can** still set the alliance (**START**) and switch field / robot centric (**BACK**) yourself. The speed source now starts on **TABLE**, so in a solo match **the camera must see the hive before a volley will start**. If the camera cannot see it, someone has to press gamepad 2 **LOGO** to switch to **DIAL** before the match. LOGO is known to reach the Robot Controller on our controllers; the INIT screen's **BUTTON TEST** shows every button each gamepad reports if you want to check.
- **Not tested on a robot yet:** whether the last ball leaves with the Y lift, whether pausing the intake when the turret is off feels right (a flickering camera pauses it for a moment), and whether a ball gets caught when the gate closes. The flywheel RPM and the ratio stay **frozen for as long as you hold RT**, so if you move to a different spot, let go and press RT again.

### Driver tips
- The robot starts **FIELD-CENTRIC**: pushing the left stick forward moves the robot **away from you**, even after the robot turns. The screen shows **DRIVE MODE** right under the alliance, and says which wall "forward" points at (**BLUE driver station: toward the LEFT wall**, **RED: toward the RIGHT wall**). **BACK** (yours) or the operator's **LOGO** switches to **ROBOT-CENTRIC**, where forward is the way the robot is facing.
- Field-centric assumes the **blue drivers stand on the right and the red drivers on the left**, and that the robot starts TeleOp **facing the way it started Auto** (blue down the field, red up the field). If the robot was turned or bumped, or you start a practice run another way, press **D-pad Left** (re-aim).
- **Safety:** if the Pinpoint stops being READY **for more than a quarter of a second**, or its heading jumps, the drive switches itself to **ROBOT-CENTRIC** and the screen says why. It does **not** switch back by itself (a restarted Pinpoint has a new zero). A shorter blink (a bad read, a weak-battery dip) is forgiven: the drive keeps the last heading and carries on, and the screen counts it under **Pinpoint glitches**. Lots of glitches means **change the battery**. **BACK**, the operator's **LOGO** or your **D-pad Left** turns field-centric back on. Do **not** change the alliance after START to fix it: the alliance only changes the camera's hive then.
- **Keep the robot still during INIT.** The Pinpoint odometry calibrates its gyro then. Before START, check that the screen says **Pinpoint status: READY**.
- **The turret keeps pointing at the hive while you spin, even if the camera loses it.** The Pinpoint remembers which way the hive was. It gives up (screen: **Hive memory ... STALE**, turret holds still) if you drive more than **12 inches** or **5 seconds** pass without the camera seeing the hive. When the camera sees the hive again it takes over.
- The turret **always tracks the hive**, so you don't need to hold anything to aim. If the screen says **NO TARGET**, the turret just holds where it is, so check the alliance and point the robot toward the hive.
- The operator's **A** (hold) floats the turret, so don't be surprised if the turret stops following while they hold it.
- **The shooter gate is the sequence's job during a volley**, and the operator's (X / B / Y on gamepad 2) between volleys. Your X / B / Y do nothing.
- You do not wait for "OK to shoot" any more: the volley waits for you. It checks the **flywheel only** (within 25 RPM for 0.15 s), plus the turret. The screen also shows the **hood wheel's error** and the old **"OK to shoot?"** line (both wheels), for information.
- **There is no way to reverse the shooter wheels** from the controllers. If a ball jams in the wheels, stop the robot and clear it by hand (power off first). The intake REVERSE (your Left Bumper) still works for the intake.

<div style="page-break-after: always"></div>

## Gamepad 2: OPERATOR

The operator **fires** (Right Trigger), sets the shooter speeds (RPM), moves the shooter gate between volleys, and picks the alliance. **The intake is the driver's**, not the operator's. **The operator does not aim the turret by hand.** **Every button always does the same thing** (no layers).

| Button | What it does |
|---|---|
| **Left Stick** (move) | — |
| **Right Stick** (move) | — |
| **A** (hold) | **FLOAT the turret** so you can turn it by hand. Let go and it goes back to tracking the hive |
| **B** | Shooter gate **OPENED** (lets the ball through). **Only between volleys** |
| **X** | Shooter gate **CLOSED** (holds the ball back). **Only between volleys** |
| **Y** (HOLD) | Shooter gate **LIFTED**, to push the very last ball into the flywheel. **Hold it while the volley is feeding** (RT still held): let go and the gate goes back to OPENED. **Between volleys:** let go and the gate closes gently. Ignored while the volley is only waiting. The firing sequence never lifts by itself |
| **Left Bumper (LB)** | — (the intake is the driver's only) |
| **Right Bumper (RB)** | — (the intake is the driver's only) |
| **Left Trigger (LT)** | — |
| **Right Trigger (RT)** (hold) | **FIRE a volley** (see *The firing sequence* on the driver page). The turret is already tracking the hive. The wheels use the flywheel speed from the **TABLE** (the camera; **the default**) or the **DIAL** (starts at flywheel 1300 RPM, hood wheel 2600 RPM), whichever you picked. **It keeps feeding while you hold**; let go to END it: the gate closes and the intake stops. Unless PRE-REV is on, both wheels go to 0 power (they coast down). **No one-volley limit** |
| **D-pad Up** | Flywheel target **UP** 100 RPM |
| **D-pad Down** | Flywheel target **DOWN** 100 RPM |
| **D-pad Left** | Ratio (hood wheel / flywheel) **DOWN** 0.05 |
| **D-pad Right** | Ratio (hood wheel / flywheel) **UP** 0.05 |
| **Start** | Switch the **alliance** BLUE / RED. It starts as **BLUE**. **Check it on the screen** before the match: the wrong alliance aims at the other hive. The driver's Start does the same |
| **Back** | **Drive toggle**: switch the drive **FIELD-CENTRIC / ROBOT-CENTRIC**. It starts **FIELD-CENTRIC**. If the safety rule switched field-centric off, this turns it back on. The driver's **Back** does the same |
| **Guide** (center **LOGO** button) | Switch the flywheel speed source **TABLE / DIAL**. It starts on **TABLE**. (On the *driver's* controller the same LOGO button is the SOLO MODE toggle, a different job.) |

**Why BACK, and not the physical MODE button?** The code cannot read the Logitech MODE button: the FTC SDK's `Gamepad` class has no field for it, and Logitech says MODE is a hardware switch that swaps the D-pad and the left stick. The INIT screen has a **BUTTON TEST** that lists every button each gamepad reports, if you want to see for yourself. If a button there ever changes when you press MODE, tell us which one and the toggle can move onto it (one line each in `BiobuzzTeleOp`: `isDriveToggleButtonPressedOnDriverPad()` and `isDriveToggleButtonPressedOnOperatorPad()`).

One move per press for the gate, the dials, Start, Back and the drive toggle. **Holding a button does not repeat it.** Press again for the next step.

**Gate priority:** if more than one gate button is pressed at the same time, **CLOSED wins, then OPENED, then LIFTED**. While a volley is running, X and B do nothing: the firing sequence runs the gate (Y still lifts it while feeding, see above).

### The turret: ALWAYS-TRACK
The turret follows the hive **all the time**, with no button held, and it also follows the robot's spin. There is **no switch** for it any more: it is always ON. It never moves during INIT.
- **A (hold)** floats the turret so you can turn it by hand (for example, to line it up before a match). Let go and it goes back to tracking.
- If the camera loses the hive, the turret **holds** where it is. It does not search.
- **Check the alliance (Start button, on either gamepad) before START.** The wrong alliance tracks the other team's hive.
- The turret will not go past **±120 degrees** (see `TurretSubsystem`). **0 degrees** is where the turret pointed when the Control Hub powered on, because the home switch is not installed yet. Power on with the turret pointing straight ahead, or the limits will be measured from the wrong place.

### Alliance and speed source
**DIAL** is the speed the operator dialed in (below), and needs no camera. **TABLE** uses the shot table: while the camera sees the hive, the flywheel speed comes from how high the hive's tags appear (closer = lower RPM). If the camera loses the hive, the table's last speed is kept for **1 second**, then the flywheel goes back to the dial. The screen says which one is in use. The hood wheel is always the flywheel speed x the ratio dial.

**Pairing warning:** gamepad 1 is paired with **Start + A** and gamepad 2 with **Start + B**. If you pair or re-pair either one while this OpMode is running, **Start flips the alliance** (it may flip to the wrong one), and **B opens the gate** (on gamepad 2). Pair before START, or check the **Alliance** line and the gate on the screen afterward.

### Shooter speeds (the dials)
These set the RPMs the **Right Trigger** shot uses. The wheels only spin while the trigger is held (or PRE-REV is on), and the shooter's speed control holds each wheel at its target RPM. **During a volley the dials are frozen** (the flywheel RPM and the ratio keep their values until the volley ends), so turning a dial mid-volley does nothing until it is over.

Starting dials: flywheel **1300 RPM** (lowest 500, highest 1450), ratio **2.00** (lowest 1.00, highest 3.00), so the hood wheel starts at **2600 RPM**. 1300 is the speed that looked about right on the robot. It is also the speed used in TABLE mode when the camera has not seen the hive lately. In TABLE mode the dial does not set the flywheel speed, but the ratio dial still sets the hood wheel.
Hood wheel target RPM = flywheel RPM x ratio. The flywheel is the **1620 RPM** motor, so the shooter will not take it past **1458 RPM** (`FLYWHEEL_MAX_TARGET_RPM` in `ShooterSubsystem`), and the dial stops at 1450. The hood wheel is a 6000 RPM motor with a limit of 5400. If either wheel would go past its limit, the screen says **CAPPED** and the real ratio is lower than the dial says. At ratio 2.00 the flywheel is always the one that runs out first.
The shooter wheels **cannot be reversed** from the controllers any more (the old operator bumper jam-clearing was removed in issue #74).

### Operator tips
- The intake is the **driver's only**. Your bumpers do nothing.
- The gate positions are tuned in `ShooterServoSubsystem` (`CLOSED_POSITION`, `OPENED_POSITION`, `LIFTED_POSITION`). The screen shows the position we **commanded**: a servo cannot report where it really is.
- **Right now free (blank) on gamepad 2:** both sticks, both bumpers, Left Trigger. **Free on gamepad 1:** A, B, X, D-pad Down, D-pad Right. (Gamepad 1's Right Trigger and Y only do something in SOLO MODE, its D-pad Up is PRE-REV, Start is the alliance, Back is the drive toggle and LOGO is the SOLO MODE toggle.)
