# BIOBUZZ OpMode Controls: Every TeleOp and Test OpMode

Print this and keep it at the driver station. It lists **what every button does in every OpMode**, using the Logitech controller names. The full TeleOp sheets (driver page and operator page) are in `biobuzz-teleop-controls.md`.

**Controller:** Logitech F310 / F710, switch on the back set to **X** (XInput), not D.
- Face buttons: **A, B, X, Y**. Bumpers: **LB / RB**. Triggers: **LT / RT**. Stick buttons: **LS / RS** (press the stick in).
- **Start** and **Back** are in the middle. (The old PS5 "Share" button is **Back**.)
- **Pairing:** **Start + A** pairs gamepad 1. **Start + B** pairs gamepad 2. Pairing while an OpMode is running also presses Start and A or B in that OpMode, so check the alliance and any saved rows afterward.

**Names on the Driver Station:** `TeleOp - Alpha 7`, `Shooter RPM - Alpha`, `Shooter Power - Alpha`, `Shooter Ratio - Alpha`, `Shot Table - Alpha`, `Turret Testing`, `Vision - Alpha`. **Hidden** (`@Disabled`, remove that line and redeploy to bring them back): `Drive Alpha`, `Intake Testing - Alpha`.

**The same button does different jobs in different OpModes.** Check the OpMode name before you press anything.

---

## 1. TeleOp - Alpha 7 (the real TeleOp)

Two controllers. The turret **tracks the hive all the time** (ALWAYS-TRACK, starts ON).

### Gamepad 1: DRIVER

| Control | What it does |
|---|---|
| **Left Stick** | Drive forward / backward and strafe left / right |
| **Right Stick** | Turn |
| **LT** (hold) | **Slow mode** |
| **RB** | **Intake** (pull balls in) |
| **LB** | **Reverse the intake** (push balls out). Beats RB |
| **RT** (hold) | **Shoot** at the dialed RPMs. It also auto-aligns the turret, even if ALWAYS-TRACK is off. **Let go and both wheels go to 0 power** |

### Gamepad 2: OPERATOR

| Control | What it does |
|---|---|
| **Left Stick X** | Turn the turret by hand (push right, it turns right). Beats tracking while pushed |
| **Y / X / B** | Turret to **center (0°)** / **far left (+90°)** / **far right (-90°)**. Each one also turns **ALWAYS-TRACK off** |
| **A** | **Stop** the turret (it floats). Also turns ALWAYS-TRACK off |
| **RS** (press the stick in) | Switch **ALWAYS-TRACK** ON / OFF (starts ON) |
| **D-pad Up / Down** | Flywheel target RPM **+ / - 100** (range 500 to 1450) |
| **D-pad Right / Left** | Hood / flywheel **ratio + / - 0.05** (range 1.00 to 3.00, starts 2.00) |
| **LB** (hold) | **Reverse BOTH shooter wheels** (clear a jam) |
| **RB** (hold) | Reverse the **flywheel only** |
| **RT** | Gate to **SHOOTING** (lets the ball through) |
| **LT** | Gate to **LOADING** (holds the ball back) |
| **Start** | Flywheel speed source **DIAL / TABLE** (starts DIAL) |
| **Back** | **Alliance** BLUE / RED (starts **BLUE**, check it on the screen) |

**Starts at:** flywheel 1300 RPM, ratio 2.00, so the hood wheel is 2600 RPM. The screen shows loop time, the alliance, and **OK to shoot?** (YES only when both wheels are at their target RPM).

---

## 2. Shooter RPM - Alpha (gamepad 1 only)

Speed-control version of the shooter test. **Hold** a button, the wheels run, let go and they coast.

| Control | What it does |
|---|---|
| **Y** (hold) | Flywheel **1400** RPM, hood wheel **2800** RPM |
| **X** (hold) | Flywheel **1300** RPM, hood wheel **2600** RPM. If Y and X are both held, Y wins |
| **RT** (hold) | **Encoder check:** both motors at FULL power, no speed control. Wheels clear. Expect about **6,000 RPM** hood wheel and **1,500 to 1,620 RPM** flywheel |
| **RB** (hold) | Reverse the **flywheel only** |
| **LB** (hold) | Reverse the **hood wheel only** |
| **LT** | **Toggle** the gate LOADING / SHOOTING |
| **D-pad Up / Down** | **Nudge the gate** position + / - 0.01 (Up raises the number) |

Priority, highest first: bumpers, RT (encoder check), Y / X.

---

## 3. Shooter Power - Alpha (two controllers)

Plain **power**, no speed control. Note: with the 1620 RPM flywheel motor, the same power is now a **much slower** flywheel, so these are not the old shots.

**Gamepad 1: shooter and gate**

| Control | What it does |
|---|---|
| **Y** (hold) | Flywheel **60%** and hood wheel **60%** power |
| **X** (hold) | Flywheel **55%** and hood wheel **55%** power. If both held, the higher power wins |
| **RB** (hold) | Reverse the **flywheel only** |
| **LB** (hold) | Reverse the **hood wheel only** |
| **LT** | **Toggle** the gate LOADING / SHOOTING |
| **D-pad Up / Down** | **Nudge the gate** position (small step) |

**Gamepad 2: intake**

| Control | What it does |
|---|---|
| **RB** (hold) | **Intake** at the dial power |
| **LB** (hold) | **Reverse** the intake. Beats RB |
| **D-pad Up / Down** | Intake power dial **+ / - 5%** |

---

## 4. Shooter Ratio - Alpha (two controllers)

Finds the best **hood / flywheel ratio**. Shoot only when the screen says **At Speed? true**. Do not record a shot marked **CAPPED**.

**Gamepad 1: shooter, gate, dials**

| Control | What it does |
|---|---|
| **B / A** | Flywheel target RPM **up / down** by 100 (range 500 to 1450, starts 1300) |
| **D-pad Right / Left** | Ratio **up / down** by 0.05 |
| **Y** | Shooter **ON** (spins at the dialed speeds and keeps going) |
| **X** | Shooter **OFF** (wheels coast down, wait for them to stop) |
| **RB** (hold) | Reverse the **flywheel only** |
| **LB** (hold) | Reverse the **hood wheel only** |
| **LT** | **Toggle** the gate LOADING / SHOOTING |
| **D-pad Up / Down** | **Nudge the gate** position |
| **Back** | Reset the **lowest RPM** readings |

**Gamepad 2: intake**

| Control | What it does |
|---|---|
| **RB** | **Intake** |
| **LB** | **Reverse** the intake |

Careful: **D-pad Right / Left** is the ratio here, and **D-pad Up / Down** is the gate nudge.

---

## 5. Shot Table - Alpha (gamepad 1 only)

Measures and checks the shot table. **No drive, no turret, no gate.** Move the robot **by hand**, load balls by hand. The intake feeds only when the screen says **READY**.

| Control | What it does |
|---|---|
| **RT** (hold) | Shoot at the **DIAL** speed |
| **LT** (hold) | Shoot at the **TABLE** speed (from the camera's `ty` right now) |
| **D-pad Up / Down** | Dial **+ / - 25 RPM** (range 500 to 1450, starts 1300) |
| **A** | **Save a row** (current `ty` + dial RPM) |
| **B** | **Clear** the saved rows |
| **LB** (hold) | Reverse **both wheels AND the intake**. Beats everything |
| **Start** | **Alliance** BLUE / RED (starts BLUE). Pairing with Start + A **also saves a row** |

Priority: LB, then RT, then LT. Copy the saved rows into `ShotTable` before you stop. Nothing is kept.

---

## 6. Turret Testing (gamepad 2 only)

| Control | What it does |
|---|---|
| **Left Stick X** | Turn the turret by hand (push right, it turns right) |
| **D-pad Left / Right** | **Hold** to turn left / right |
| **D-pad Up / Down** | Turret power **+ / - 5%** (starts 100%) |
| **Y** | Preset: **center (0°)** |
| **X** | Preset: **far left (+90°)** |
| **B** | Preset: **far right (-90°)** |
| **A** | **Stop** (turret floats) |

Priority: stick, D-pad Left / Right, Y / X / B, then A. Letting go of the stick or D-pad holds the turret. Angle limits are **+/-120°**, measured from where the turret pointed when the Control Hub powered on. Power on with the turret **straight ahead**.

---

## 7. Vision - Alpha

Step 1 is camera only (no controls, just INIT and watch). Step 2 aims the turret.

| Control | What it does |
|---|---|
| **Gamepad 1 Start** | **Alliance** BLUE / RED (starts BLUE) |
| **Gamepad 2 RB** (hold) | **Auto-align** the turret on the hive |
| **Gamepad 2 Left Stick X** | Turn the turret by hand. Beats auto-align |
| **Gamepad 2 A** | **Stop** the turret. Beats everything |

Letting go of RB or the stick holds the turret.

---

## 8. Hidden OpModes (`@Disabled`)

**Drive Alpha** (gamepad 1)

| Control | What it does |
|---|---|
| **Left Stick** | Drive and strafe |
| **Right Stick** | Turn |
| **RT** (hold) | **Slow mode** (this OpMode only. The TeleOp uses **LT**) |
| **X / A / Y / B** (hold) | Wheel test, robot **on blocks**: **X** left front, **A** left rear, **Y** right front, **B** right rear. Each must push the robot forward |

**Intake Testing - Alpha** (gamepad 1)

| Control | What it does |
|---|---|
| **RB** (hold) | **Intake** at the dial power |
| **LB** (hold) | **Reverse**. Beats RB |
| **D-pad Up / Down** | Power dial **+ / - 5%** |

---

## Quick "which OpMode am I in" differences

| Job | TeleOp Alpha 7 | The test OpModes |
|---|---|---|
| Shoot | Driver **RT** (hold) | **Y / X** hold (RPM and Power Alpha), **Y** on / **X** off (Ratio Alpha), **RT / LT** (Shot Table) |
| Gate | Operator **RT** shoot / **LT** load | **LT** toggles (Shooter alphas), none (Shot Table) |
| Flywheel speed | Operator **D-pad Up / Down** | **B / A** (Ratio Alpha), **D-pad Up / Down** (Shot Table) |
| Alliance | Operator **Back** | **Start** (Vision, Shot Table) |
| Slow mode | Driver **LT** | Driver **RT** (Drive Alpha) |

If a button here doesn't match the robot, the code changed. Update this sheet.
