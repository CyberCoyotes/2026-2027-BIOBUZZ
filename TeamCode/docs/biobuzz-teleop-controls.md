# BIOBUZZ TeleOp Controls: Driver and Operator

Print this and keep it at the driver station. It matches **`TeleOp - Alpha 5`** (`BiobuzzTeleOp.java`). If the code changes, update this sheet.

**Pairing:** on the Driver Station, press **Start + A** on the driver's controller (gamepad 1) and **Start + B** on the operator's controller (gamepad 2).
**Logitech controller (F310 / F710):** set the switch on the back to **X** (XInput mode), not D. The buttons are then labeled A, B, X, Y like the code. Bumpers are **LB / RB** and triggers are **LT / RT**. Left Trigger = LT, Right Trigger = RT.

**Safety:** keep hands, hair, and loose clothing away from the shooter wheels, and keep everyone out of the path of the ball. Keep a hand near the **Stop** button while the turret limits are off.

## Gamepad 1: DRIVER

The driver drives, runs the intake, flips the shooter gate, and can fire a fixed shot.

| Control | What it does |
|---|---|
| **Left Stick** | Drive forward / backward and slide (strafe) left / right |
| **Right Stick** | Turn (push right to turn right) |
| **Left Trigger** (hold) | **SLOW MODE** (40% speed) for careful lining up |
| **Right Bumper** | **INTAKE**: pull balls in (also carries them up the transfer chute) |
| **Left Bumper** | **REVERSE** the intake: push balls out. Beats the Right Bumper, so clearing a jam always works |
| **Right Trigger** (hold) | **FIXED SHOT**: flywheel 45%, hood wheel 90%. **Let go and both wheels go to 0 power** (they coast down) |
| **A** | **TOGGLE the shooter gate** between LOADING and SHOOTING (one flip per press) |

### Driver tips
- The robot is **robot-centric**: "forward" is the way the robot is facing.
- Holding A does **not** keep flipping the gate. One press is one flip.
- The operator's bumpers (jam clearing) beat your Right Trigger shot, so don't be surprised if the shot stops.
- Wait for **"Wheels settled? YES - OK to shoot"** on the screen before you open the gate. After the shooter starts, a dial change, a reverse, or any gate flip, the screen counts 3 seconds again.

<div style="page-break-after: always"></div>

## Gamepad 2: OPERATOR

The operator runs the shooter wheels and aims the turret. **Gamepad 2 has two layers**, because the shooter and the turret both want the same buttons.

> **Hold the Left Trigger** = TURRET layer. **Let go** = SHOOTER layer.
> The screen shows **"Gamepad2 layer"** so you can check which one you are in.

### Shooter layer (Left Trigger NOT held)

| Control | What it does |
|---|---|
| **Y, X, B, A** | **No shooter job.** They only run the turret, in the turret layer below |
| **D-pad Right** | Ratio **UP** 0.05 |
| **D-pad Left** | Ratio **DOWN** 0.05 |

The shooter wheels now run **only** from the driver's Right Trigger shot (or the bumpers, which reverse them).

### Turret layer (HOLD Left Trigger)

In this layer the ratio dial (D-pad Left / Right) is ignored, because D-pad Left / Right turns the turret instead.

| Control | What it does |
|---|---|
| **Y** | Turret **CENTER** (0 degrees) |
| **X** | Turret **FAR LEFT** (+90 degrees) |
| **B** | Turret **FAR RIGHT** (-90 degrees) |
| **A** | **STOP** the turret (it floats, so you can turn it by hand) |
| **D-pad Left / Right** | **HOLD** to turn the turret left / right |

A preset goes to its angle and holds there.

### Works in BOTH layers

| Control | What it does |
|---|---|
| **Left Stick X** | Turn the turret (push right and it turns right). A full push is the whole turret power |
| **D-pad Up / Down** | Turret power **+5% / -5%** (starts at 50%, range 0 to 100%) |
| **Right Bumper** (hold) | **Reverse the flywheel** only (clearing a jam) |
| **Left Bumper** (hold) | **Reverse the hood wheel** only (clearing a jam) |

Bumpers beat everything else on the shooter, including the driver's Right Trigger shot.

Letting go of the stick or the D-pad **holds the turret where it is**.

### Operator tips
- **Turret limits are OFF for now** (see `TurretSubsystem`). Keep a hand near the Stop button.
- **0 degrees** is where the turret pointed when the Control Hub powered on, because the home switch is not installed yet. Power on with the turret pointing straight ahead.
- To aim while tuning the shooter, use the **Left Stick** or **D-pad Up / Down**. Those work without holding the trigger.
- Let go of the Left Trigger to go back to the shooter. A button still held when you let go does not count as a new press.
