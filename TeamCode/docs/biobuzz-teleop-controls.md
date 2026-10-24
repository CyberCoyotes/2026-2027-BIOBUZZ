# BIOBUZZ TeleOp Controls: Driver and Operator

Print this and keep it at the driver station. It matches **`TeleOp - Alpha 5`** (`BiobuzzTeleOp.java`). If the code changes, update this sheet.

**Pairing:** on the Driver Station, press **Start + A** on the driver's controller (gamepad 1) and **Start + B** on the operator's controller (gamepad 2).
**Logitech controller (F310 / F710):** set the switch on the back to **X** (XInput mode), not D. The buttons are then labeled A, B, X, Y like the code. Bumpers are **LB / RB** and triggers are **LT / RT**. Left Trigger = LT, Right Trigger = RT.

**Safety:** keep hands, hair, and loose clothing away from the shooter wheels, and keep everyone out of the path of the ball. Keep a hand near the **Stop** button when testing the turret.

## Gamepad 1: DRIVER

The driver drives, runs the intake, and holds the Right Trigger to shoot.

| Control | What it does |
|---|---|
| **Left Stick** | Drive forward / backward and slide (strafe) left / right |
| **Right Stick** | Turn (push right to turn right) |
| **Left Trigger** (hold) | **SLOW MODE** (40% speed) for careful lining up |
| **Right Bumper** | **INTAKE**: pull balls in (also carries them up the transfer chute) |
| **Left Bumper** | **REVERSE** the intake: push balls out. Beats the Right Bumper, so clearing a jam always works |
| **Right Trigger** (hold) | **SHOOT** at the RPMs the operator dialed in (starts at flywheel 2700 RPM, hood wheel 5400 RPM). **Let go and both wheels go to 0 power** (they coast down) |

### Driver tips
- The robot is **robot-centric**: "forward" is the way the robot is facing.
- The operator's bumpers (jam clearing) beat your Right Trigger shot, so don't be surprised if the shot stops.
- Wait for **"OK to shoot? YES"** on the screen before the operator opens the gate. It turns YES only when **both wheels are at their target RPM** (the screen shows each wheel's real RPM next to its target). After a ball slows the wheels, it goes back to WAIT until they recover.

<div style="page-break-after: always"></div>

## Gamepad 2: OPERATOR

The operator sets the shooter speeds (RPM), aims the turret, and moves the shooter gate. **Every button always does the same thing** (no layers).

### Turret

| Control | What it does |
|---|---|
| **Left Stick X** | Turn the turret (push right and it turns right). A full push is 50% turret power |
| **Y** | Turret **CENTER** (0 degrees) |
| **X** | Turret **FAR LEFT** (+90 degrees) |
| **B** | Turret **FAR RIGHT** (-90 degrees) |
| **A** | **STOP** the turret (it floats, so you can turn it by hand) |

A preset goes to its angle and holds there. Letting go of the stick **holds the turret where it is**.

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

Starting dials: flywheel **2700 RPM** (lowest 500, highest 3600), ratio **2.00** (lowest 1.00, highest 3.00), so the hood wheel starts at **5400 RPM**. These starting numbers are estimates (the old 45% power x 6000 RPM). Tune them on the robot.
Hood wheel target RPM = flywheel RPM x ratio. The shooter will not go past 5400 RPM on a wheel (`MAX_TARGET_RPM` in `ShooterSubsystem`), so if the hood wheel would, the screen says **CAPPED** and the real ratio is lower than the dial says. At ratio 2.00 that means the flywheel can't go above 2700 RPM.
Jam clearing (the bumpers) is plain reverse power, not RPM. Bumpers beat everything else on the shooter, including the driver's Right Trigger shot.
Holding a D-pad button moves the dial **one step**. Press it again for the next step.

### Operator tips
- **The turret will not go past +/-120 degrees** (see `TurretSubsystem`). At a limit it stops, and you can still turn it back toward the middle.
- **0 degrees** is where the turret pointed when the Control Hub powered on, because the home switch is not installed yet. Power on with the turret pointing straight ahead, or the limits will be measured from the wrong place.
- The turret power is fixed at 50% now. It no longer has a dial.
