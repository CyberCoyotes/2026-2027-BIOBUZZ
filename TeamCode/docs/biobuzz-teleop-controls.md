# BIOBUZZ TeleOp Controls: Driver and Operator

Print this and keep it at the driver station. It matches **`TeleOp - Alpha 7`** (`BiobuzzTeleOp.java`). If the code changes, update this sheet. The controls for every other OpMode (the test Alphas) are in `biobuzz-opmode-controls.md`.

**Pairing:** on the Driver Station, press **Start + A** on the driver's controller (gamepad 1) and **Start + B** on the operator's controller (gamepad 2).
**Logitech controller (F310 / F710):** set the switch on the back to **X** (XInput mode), not D. The buttons are then labeled A, B, X, Y like the code. Bumpers are **LB / RB** and triggers are **LT / RT**. Left Trigger = LT, Right Trigger = RT.

**Safety:** keep hands, hair, and loose clothing away from the shooter wheels, and keep everyone out of the path of the ball. Keep a hand near the **Stop** button when testing the turret.

## Gamepad 1: DRIVER

The driver drives, runs the intake, and holds the Right Trigger to shoot. **The Right Trigger also aims the turret at the hive for you.**

| Control | What it does |
|---|---|
| **Left Stick** | Drive forward / backward and slide (strafe) left / right |
| **Right Stick** | Turn (push right to turn right) |
| **Left Trigger** (hold) | **SLOW MODE** (40% speed) for careful lining up |
| **Right Bumper** | **INTAKE**: pull balls in (also carries them up the transfer chute) |
| **Left Bumper** | **REVERSE** the intake: push balls out. Beats the Right Bumper, so clearing a jam always works |
| **Right Trigger** (hold) | **SHOOT** (it also auto-aligns the turret, even if ALWAYS-TRACK is off). The wheels use the flywheel speed from the **DIAL** (starts at flywheel 1300 RPM, hood wheel 2600 RPM) or the **TABLE**, whichever the operator picked. **Let go and both wheels go to 0 power** (they coast down), and the turret holds where it ended |

### Driver tips
- The robot is **robot-centric**: "forward" is the way the robot is facing.
- The operator's bumpers (jam clearing) beat your Right Trigger shot, so don't be surprised if the shot stops.
- Auto-align needs the camera to see the hive. If the screen says **NO TARGET**, the turret just holds where it is, so check the alliance and point the robot toward the hive.
- The turret now **tracks the hive all the time** (**ALWAYS-TRACK**, see the operator page), so you don't need to hold anything to aim. The screen shows **ALWAYS-TRACK ON / OFF**.
- The operator's turret stick and **A (STOP)** beat the auto-align.
- Wait for **"OK to shoot? YES"** on the screen before the operator opens the gate. It turns YES only when **both wheels are at their target RPM** (the screen shows each wheel's real RPM next to its target). After a ball slows the wheels, it goes back to WAIT until they recover.

<div style="page-break-after: always"></div>

## Gamepad 2: OPERATOR

The operator sets the shooter speeds (RPM), aims the turret, and moves the shooter gate. **Every button always does the same thing** (no layers).

### Turret

| Control | What it does |
|---|---|
| **Left Stick X** | Turn the turret (push right and it turns right). A full push is 100% turret power |
| **Y** | Turret **CENTER** (0 degrees) |
| **X** | Turret **FAR LEFT** (+90 degrees) |
| **B** | Turret **FAR RIGHT** (-90 degrees) |
| **A** | **STOP** the turret (it floats, so you can turn it by hand) |
| **Right Stick Button** (press the stick in) | Switch **ALWAYS-TRACK** ON / OFF (see below). It starts **ON** |

A preset goes to its angle and holds there.

**ALWAYS-TRACK:** the turret follows the hive **all the time**, with no button held (it also follows the robot's spin). It starts **ON**, and it never moves during INIT.
- **Stick:** pushing the **Left Stick** beats it. Let go and it goes back to tracking.
- **Y, X, B and A switch it OFF**, so the turret stays where you put it. Press the **Right Stick Button** to turn it back ON.
- If the camera loses the hive, the turret **holds** where it is. It does not search.
- **Check the alliance (Back button) before START.** The wrong alliance tracks the other team's hive.

### Alliance and speed source

| Control | What it does |
|---|---|
| **Back** | Switch the **alliance** BLUE / RED. It starts as **BLUE**. **Check it on the screen** before the match: the wrong alliance aims at the other hive |
| **Start** | Switch the flywheel speed source **DIAL / TABLE**. It starts on **DIAL** |

**DIAL** is the speed the operator dialed in (below), exactly as before, and needs no camera. **TABLE** uses the shot table: while the camera sees the hive, the flywheel speed comes from how high the hive's tags appear (closer = lower RPM). If the camera loses the hive, the table's last speed is kept for **1 second**, then the flywheel goes back to the dial. The screen says which one is in use. The hood wheel is always the flywheel speed x the ratio dial.
(Start + B is also how gamepad 2 is paired, so pairing while the OpMode is running switches the mode. Check it on the screen.)

### Shooter gate

| Control | What it does |
|---|---|
| **Right Trigger** | Gate to **SHOOTING** (lets the ball through) |
| **Left Trigger** | Gate to **LOADING** (holds the ball back) |

One move per press. If both are pressed at once, LOADING wins.

### Shooter speeds (the dials)

These set the RPMs the driver's **Right Trigger** shot uses. The wheels only spin while the driver holds the trigger, and the shooter's speed control holds each wheel at its target RPM.

| Control | What it does |
|---|---|
| **D-pad Up** | Flywheel target **UP** 100 RPM |
| **D-pad Down** | Flywheel target **DOWN** 100 RPM |
| **D-pad Right** | Ratio (hood wheel / flywheel) **UP** 0.05 |
| **D-pad Left** | Ratio **DOWN** 0.05 |
| **Left Bumper** (hold) | **Reverse BOTH wheels** (flywheel and hood wheel) to clear a jam |
| **Right Bumper** (hold) | Reverse the **flywheel** only |

Starting dials: flywheel **1300 RPM** (lowest 500, highest 1450), ratio **2.00** (lowest 1.00, highest 3.00), so the hood wheel starts at **2600 RPM**. 1300 is the speed that looked about right on the robot. It is also the speed used in TABLE mode when the camera has not seen the hive lately. In TABLE mode the dial does not set the flywheel speed, but the ratio dial still sets the hood wheel.
Hood wheel target RPM = flywheel RPM x ratio. The flywheel is the **1620 RPM** motor, so the shooter will not take it past **1458 RPM** (`FLYWHEEL_MAX_TARGET_RPM` in `ShooterSubsystem`), and the dial stops at 1450. The hood wheel is a 6000 RPM motor with a limit of 5400. If either wheel would go past its limit, the screen says **CAPPED** and the real ratio is lower than the dial says. At ratio 2.00 the flywheel is always the one that runs out first.
Jam clearing (the bumpers) is plain reverse power, not RPM. Bumpers beat everything else on the shooter, including the driver's Right Trigger shot.
Holding a D-pad button moves the dial **one step**. Press it again for the next step.

### Operator tips
- **The turret will not go past +/-120 degrees** (see `TurretSubsystem`). At a limit it stops, and you can still turn it back toward the middle.
- **0 degrees** is where the turret pointed when the Control Hub powered on, because the home switch is not installed yet. Power on with the turret pointing straight ahead, or the limits will be measured from the wrong place.
- The turret stick power is fixed at 100% now. It no longer has a dial. At full power the turret coasts further past a limit, so ease the stick toward the +/-120 degree limits until you know how far it goes.
