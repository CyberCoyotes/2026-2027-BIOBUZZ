# BIOBUZZ TeleOp Controls: Driver and Operator

Print this and keep it at the driver station. It matches **`TeleOp - Alpha 5`** (`BiobuzzTeleOp.java`). If the code changes, update this sheet.

**Pairing:** on the Driver Station, press **Start + A** on the driver's controller (gamepad 1) and **Start + B** on the operator's controller (gamepad 2).
**Logitech controller (F310 / F710):** set the switch on the back to **X** (XInput mode), not D. The buttons are then labeled A, B, X, Y like the code. Bumpers are **LB / RB** and triggers are **LT / RT**. Left Trigger = LT, Right Trigger = RT.

**Safety:** keep hands, hair, and loose clothing away from the shooter wheels, and keep everyone out of the path of the ball. Keep a hand near the **Stop** button while the turret limits are off.

## Gamepad 1: DRIVER

The driver drives, runs the intake, and holds the Right Trigger to shoot.

| Control | What it does |
|---|---|
| **Left Stick** | Drive forward / backward and slide (strafe) left / right |
| **Right Stick** | Turn (push right to turn right) |
| **Left Trigger** (hold) | **SLOW MODE** (40% speed) for careful lining up |
| **Right Bumper** | **INTAKE**: pull balls in (also carries them up the transfer chute) |
| **Left Bumper** | **REVERSE** the intake: push balls out. Beats the Right Bumper, so clearing a jam always works |
| **Right Trigger** (hold) | **SHOOT** at the powers the operator dialed in (starts at flywheel 45%, hood wheel 90%). **Let go and both wheels go to 0 power** (they coast down) |

### Driver tips
- The robot is **robot-centric**: "forward" is the way the robot is facing.
- The operator's bumpers (jam clearing) beat your Right Trigger shot, so don't be surprised if the shot stops.
- Wait for **"Wheels settled? YES - OK to shoot"** on the screen before the operator opens the gate. After the trigger goes down or up, a dial change, a reverse, or any gate flip, the screen counts 3 seconds again.

<div style="page-break-after: always"></div>

## Gamepad 2: OPERATOR

The operator sets the shooter powers, aims the turret, and moves the shooter gate. **Every button always does the same thing** (no layers).

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

### Shooter powers (the dials)

These set the powers the driver's **Right Trigger** shot uses. The wheels only spin while the driver holds the trigger.

| Control | What it does |
|---|---|
| **D-pad Up** | Flywheel power **UP** 5% |
| **D-pad Down** | Flywheel power **DOWN** 5% |
| **D-pad Right** | Ratio (hood wheel / flywheel) **UP** 0.05 |
| **D-pad Left** | Ratio **DOWN** 0.05 |
| **Right Bumper** (hold) | **Reverse the flywheel** only (clearing a jam) |
| **Left Bumper** (hold) | **Reverse the hood wheel** only (clearing a jam) |

Starting dials: flywheel **45%** (lowest 5%, highest 100%), ratio **2.00** (lowest 1.00, highest 3.00), so the hood wheel starts at **90%**.
Hood wheel power = flywheel power x ratio. A wheel can't go past 100%, so if the hood wheel would, the screen says **CAPPED**.
Bumpers beat everything else on the shooter, including the driver's Right Trigger shot.
Holding a D-pad button moves the dial **one step**. Press it again for the next step.

### Operator tips
- **Turret limits are OFF for now** (see `TurretSubsystem`). Keep a hand near the Stop button.
- **0 degrees** is where the turret pointed when the Control Hub powered on, because the home switch is not installed yet. Power on with the turret pointing straight ahead.
- The turret power is fixed at 50% now. It no longer has a dial.
