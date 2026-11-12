# BIOBUZZ Match-Day Checklists

*DRAFT. Print these. Every line is a thing that has gone wrong, or could, based on what is in this repo. Matches `TeleOp - Alpha 8` (`BiobuzzTeleOp.java`) as of 2026-11-03. When the code changes, change this file in the same pull request. Controls are in `biobuzz-teleop-controls.md`; problems are in `biobuzz-troubleshooting.md`.*

**Rule of thumb from `CLAUDE.md`: check the cables before the code.** Motors or sensors in the wrong port waste more time than any bug.

## A. In the pit, before we queue

- [ ] Battery fully charged, a second one charged and ready. Note the voltage.
- [ ] All cables seated: drive motors, intake, flywheel, hood wheel, turret motor and encoder, gate servo, Pinpoint (I2C), Limelight (USB-C to the Control Hub's **USB 3.0** port).
- [ ] Nothing loose that could fall off (G414: team number and alliance color signs on and visible).
- [ ] Robot inside its 18 in cube in starting shape (R102) and the arm/turret fit.
- [ ] Gate servo set to **CLOSED**.
- [ ] Turret is **on the home switch, pointing straight ahead (intake direction)**. If the hub has been power-cycled, the zero is wherever the turret sat at power-up. The soft limits at +/-120 degrees are only right if the zero is right.
- [ ] Pollen (4) loaded.
- [ ] Robot config matches the code names exactly (`ARCHITECTURE.md` section 2): `pinpoint` (not last season's `odo`), `limelight` (Ethernet device), `turret_home_switch` as a **Digital Device**, plus the drive, intake, flywheel, hood wheel, turret and servo names. A wrong name stops the OpMode at INIT.
- [ ] Correct Control Hub for the robot: practice bot, 11940, or 22091.

## B. Driver Station

- [ ] Gamepads paired: **Start + A** (driver, gamepad 1), **Start + B** (operator, gamepad 2). Logitech: switch on the back is on **X**.
- [ ] The Auto OpMode is selected **with the 30 second timer on** (G305). If we are not running Auto, we still need to select a TeleOp OpMode.
- [ ] Press **INIT** on the selected OpMode.
- [ ] Wait about **30 seconds** after power-on for the Limelight to boot.

## C. At the field (G304 setup)

- [ ] Robot on **our half**, **touching the perimeter wall**, **not** in the LOADING ZONE, not touching a FLOWER.
- [ ] Exactly **4 POLLEN** touching the robot.
- [ ] Robot sitting still after INIT. **Keep it still** so the Pinpoint can calibrate its gyro.
- [ ] Screen says **Pinpoint status: READY**.
- [ ] Screen shows the **alliance we are playing**, not just BLUE by default. (TeleOp starts as BLUE; **Start** on either gamepad switches it. A wrong alliance aims at the other hive and does not look like a failure.)
- [ ] With the robot pointed at the hive, the screen shows a target (not **NO TARGET**).
- [ ] Partner talk done: Auto plans and start spots match, who parks where (see `biobuzz-auto-strategy.md` section 7).
- [ ] Human player knows to add nectar only after a tip (one per tip) until 1:00 is left.

## D. AUTO

- [ ] Start button pressed by the drive coach or a driver right at the start (G401: no other contact with the robot or the Driver Station during AUTO).
- [ ] Hands off everything.

## E. Transition (8 seconds)

- [ ] **Nothing powered moves** (G403).
- [ ] Switch to TeleOp or confirm the queued TeleOp loaded. The TeleOp INIT does not move any motor.
- [ ] Re-check the alliance on the screen.
- [ ] Press **START** when the TeleOp timer starts.

## F. TELEOP

- [ ] Alliance correct on screen. ALWAYS-TRACK on.
- [ ] The operator holds the **Right Trigger** to fire (one-person match: the driver presses **LOGO** for SOLO MODE first, then fires with the driver's Right Trigger). The sequence waits for the wheels and the turret by itself (the screen says **Waiting for** what is missing). It keeps feeding while held: let go when the balls are gone, and hold **Y** if the last ball needs a push.
- [ ] Pre-rev with the driver's **D-pad Up** while driving to the spot. Check **PRE-REV** is OFF again if you do not want the wheels spinning.
- [ ] Never more than 4 balls on the robot (G407).
- [ ] At 1:00 left, human player may add all nectar. Our robot does not need to change anything.
- [ ] At about 0:20 the drive coach calls the PARK time (see `biobuzz-game-strategy.md` section 6).
- [ ] At 0:00: **stop moving**. Press stop on the Driver Station and set the controllers down (G404).

## G. After the match

- [ ] Fill in a row in `biobuzz-test-and-match-log.md`: score, what worked, what broke, and anything strange on the screen.
- [ ] Note the **lowest voltage** of each hub and the **dip count** from the **POWER** lines on the screen. If anything blinked (the camera, the Pinpoint, the turret), or a dip count is above 0, read the log from a laptop on the robot's wifi: `adb logcat -d | grep BIOBUZZ`. Each line has the time and what the battery was doing. Write what you find in `biobuzz-test-and-match-log.md`.
- [ ] Look for loose cables, a bent mechanism, and a dented ball path.
- [ ] Swap the battery if it fell low.
- [ ] If something got changed in code at the event, commit it, so the robot and the repo agree.

## H. Before leaving an event

- [ ] All tuned numbers that changed (shot table rows, servo positions, aim offset) are written in the log and copied into the code (`ARCHITECTURE.md`, "Where constants live").
- [ ] Both robots' code pushed to GitHub.
