# BIOBUZZ Autons on the Driver Station

**Print this and keep it by the Driver Station.** **Page 1** is the list you pick from, with what each Auto does. **Page 2** is the check before a match, the Autos that are hidden, and what is still open. **Checked against the code on 2026-11-24 (Issue #118).**

The names below are copied letter for letter from the `name = "..."` text in each `@Autonomous` line, so what you see on the Driver Station should match them exactly.

**About the order:** this list is alphabetical, which is how we expect the Driver Station to show it. The real Driver Station app was not open when this sheet was written, so **look at the real screen once. If the order is different, fix it here.**

## What you pick from: 13 Autos

All 13 are in group **Auto**. Every one loads **TeleOp - Alpha 15** after it ends (`preselectTeleOp`).

```
 Autonomous                         (13 on the list)
 ----------------------------------------------------
 Blue - Drive Forward 24in
 Blue - Drive Reverse 24in
 Blue D1 - Move Park                [#112]
 Blue D1 - Shoot Park               [#112]
 Blue D6 - Move Park
 Blue D6 - Shoot Park
 Drive Forward
 Red - Drive Forward 24in
 Red - Drive Reverse 24in
 Red C1 - Move Park
 Red C1 - Shoot Park
 Red C6 - Move Park                 [#112]
 Red C6 - Shoot Park                [#112]
```

**`[#112]` means the coach flagged it "Remove or fix heading" and has not decided yet.** Do not pick one of those for a match until #112 is settled.

### Match Autos (8): the robot follows a path with Pedro Pathing

Pick the one for **our alliance and the tile we start on**. Wrong alliance means it aims at the other hive and parks on the wrong side. Blue parks in the blue loading zone (F2), red in the red loading zone (A5).

| Driver Station name | Robot starts | Faces | What it does | Points | Time (estimate) |
|---|---|---|---|---|---|
| **Blue D6 - Move Park** | Tile D6, rear wall | down, at the hive | Drives into our loading zone and stops | LEAVE 3 + PARK 5 = **8** | about 4 s |
| **Blue D6 - Shoot Park** | Tile D6, rear wall | down, at the hive | Shoots the 4 preloaded POLLEN, then parks | **8**, plus any score from the shots | about 8 to 10 s |
| **Red C1 - Move Park** | Tile C1, audience wall | up, at the hive | Same as Blue D6, turned 180 degrees | **8** | about 4 s |
| **Red C1 - Shoot Park** | Tile C1, audience wall | up, at the hive | Same as Blue D6 - Shoot Park, turned 180 degrees | **8**, plus shots | about 8 to 10 s |
| **Blue D1 - Move Park** `[#112]` | Tile D1, audience wall, about half an inch from the blue flower | **the wall**: front of the robot touching it | Backs straight off the wall (in reverse), curves over and parks. It never turns | **8** | about 4 s |
| **Blue D1 - Shoot Park** `[#112]` | Tile D1 (same) | **the wall** (same) | Backs up, turns left 120 degrees, shoots 4, then parks | **8**, plus shots | about 8 to 10 s |
| **Red C6 - Move Park** `[#112]` | Tile C6, rear wall, next to the red flower | **the wall**: front of the robot touching it | Same idea as Blue D1, turned 180 degrees | **8** | about 4 s |
| **Red C6 - Shoot Park** `[#112]` | Tile C6 (same) | **the wall** (same) | Backs up, turns left 120 degrees, shoots 4, then parks | **8**, plus shots | about 8 to 10 s |

- **Default choice: `Shoot Park` from D6 (blue) or C1 (red).** If the shooter or camera is broken but driving works, pick `Move Park`.
- The time numbers are estimates from a simulation (`auto/testing/README.md`). **How many balls it takes to tip the hive is still unknown**, so a tip is a bonus, not a promise (`biobuzz-auto-strategy.md`, section 4).
- The D1 and C6 starts are tight: the robot starts about half an inch from a flower. Place it centered in the tile and square to the wall.
- **D1 and C6 start FACING THE WALL (issue #121): the front of the robot (the intake) touches the wall, and the hive is behind the robot.** This is the opposite of D6 and C1, where the back is against the wall. The robot then faces the same way as at D6 / C1 on the field map, so the TeleOp's field-centric fallback is right for every start spot. On the INIT screen `Hive visible` is **not checked** for these two (the camera looks at the wall until the robot turns). **These changes have not run on a robot yet**: use STEP mode on the practice field first.

### Backup Autos (5): no Pedro, no camera, just drive 24 inches in a straight line

Use these if Pedro or the Pinpoint is broken, or the robot cannot get to a path start. They earn **LEAVE (3 points)** only, and they **do not park**. The 24 inches is where the robot should **end up**. Each one stops a few inches early because the robot rolls on after the wheels stop.

| Driver Station name | Put the robot like this | What it does | Tells the TeleOp the alliance? |
|---|---|---|---|
| **Blue - Drive Forward 24in** | Back against a wall | Drives forward 24 in | Yes, BLUE |
| **Red - Drive Forward 24in** | Back against a wall | Drives forward 24 in | Yes, RED |
| **Blue - Drive Reverse 24in** | **Intake** against a wall | Drives **backward** 24 in | Yes, BLUE |
| **Red - Drive Reverse 24in** | **Intake** against a wall | Drives **backward** 24 in | Yes, RED |
| **Drive Forward** | Back against a wall, facing the hive | Drives forward 24 in (backup timed drive if the Pinpoint is not ready) | **No.** The TeleOp falls back to BLUE |

- None of the four "24in" Autos has a tile in its name: they drive 24 inches from **wherever the robot starts**.
- They do **not** hand the robot's heading to the TeleOp, so field-centric drive is only right after a start with the back against the rear wall (blue) or the audience wall (red), or the intake against the audience wall (blue) or the rear wall (red). Otherwise press **D-pad Left** (re-aim) in the TeleOp.
- First test (2026-11-23): with no coast allowance it went about 27.4 in forward and 27.7 in reverse. After the coast allowance was added it went 24.1 in (the direction and the robot were not written down). Backward distances for each side are still being checked `[#115]`.

<div style="page-break-after: always"></div>

# Page 2: before the match, hidden Autos, open items

## Before the match

1. **Pick the Auto** for our alliance and our start tile (page 1). Select it **with the 30 second timer on** (G305).
2. **Robot** touching the wall, centered in the tile, facing the hive, **4 POLLEN** loaded, on our half and **not** in the LOADING ZONE.
3. **Turret pointing straight ahead when the robot is powered on.** Its zero is wherever it pointed at power-up.
4. **Power on at least 30 seconds before INIT**, so the Limelight can boot.
5. Press **INIT**, then **do not touch the robot** (INIT zeroes the Pinpoint).
6. **Read the INIT screen:**
   - Match Autos (Pedro): our **alliance**, **tile** and **routine** at the top, `Mode: NONSTOP`, `Actions: ON`, `Hive visible: YES` (Shoot Park only), and **no `NOT TUNED` warning**.
   - Backup Autos: no alliance buttons. The alliance is in the name.
7. **Gamepad 1 buttons during INIT are for testing only** (Pedro match Autos): **A** STEP mode (stops after every drive: **never in a match**, nobody can press A), **B** NONSTOP (the default), **X** drive only (shots become short pauses), **Y** shooting on (the default). With no buttons pressed it runs as a real match Auto.
8. **After the Auto:** the Driver Station loads **TeleOp - Alpha 15**. Check the **alliance** line. The eight Pedro Autos and the four "24in" Autos hand it over. Plain **Drive Forward** does not, so it shows BLUE.

## Not on the list (hidden with `@Disabled`)

The code is all still here. To bring one back, put `//` in front of its `@Disabled` line and redeploy.

| Name on the Driver Station | Group | Why it is hidden |
|---|---|---|
| Blue D6 - 3 Volley, Blue D1 - 3 Volley, Red C1 - 3 Volley, Red C6 - 3 Volley | Auto | Runs about 35 s, past the 30 s limit. Hidden 2026-11-21 |
| Leave and Park - Mark 1 | Auto | The Pedro "Move Park" Autos do the same job for each start. Hidden 2026-11-21 |
| Auto - Shoot and Park - Alpha 1 | Testing | Test Auto, nobody is using it. Moving it onto the shared firing sequence is `[#89]` |
| Pedro Forward Back - Alpha | Testing | Pedro tuning test, nobody is using it |
| 1 Volley Park Mirror - Alpha, 3 Volley Mirror - Alpha, 2 Volley Rear Flower Mirror - Alpha, 2 Volley Wall Flower Mirror - Alpha | Testing | Built for our practice-field layout. **Never run these in a match** |

## Where the files are (moved 2026-11-24)

Under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:

- **`auto/`**: the four `24in` Autos, and `Move Park` and `Shoot Park` from **Blue D6** and **Red C1**.
- **`auto/testing/`**: `BiobuzzAuto` (the engine the Pedro Autos share: **do not delete this folder**), the **Blue D1** and **Red C6** Autos, plain **Drive Forward** (`AutoEmergencyLeave`), and the hidden 3 Volley and `Leave and Park` Autos.
- **A folder named `testing` does not hide an Auto.** Only `@Disabled` does. Today **Blue D1, Red C6 and Drive Forward are in `auto/testing/` and are still on the Driver Station list.**

## Open items that change this sheet

- **`[#112]` Remove the broken autons.** The coach flagged the four D1 and C6 Autos "Remove or fix heading". The D6 and C1 Autos and plain **Drive Forward** are on the same review list with no problem written next to them. The note on Drive Forward ("make a blue and red") was answered by the four "24in" Autos (#114), so plain Drive Forward may be next to retire. When #112 is closed, take the markers out of this sheet and hide or remove the Autos it names.
- **`[#115]` Backward distances for each side** are still being validated.
- **Robot testing of the match Autos:** Pedro was tuned on 2026-11-17 (`FORESIGHT_TUNED` is `true` in `pedro/Constants.java`, and #57 was closed on 2026-11-18). The Autos README (written 2026-11-20) says none of the 8 has run on a real field, and no later result is written down. **Run each one you might use in STEP mode on the event's practice field before a match.** If there is no chance to test, use **Shoot Park** or **Move Park** from D6 or C1, or a **24in** backup.

## Keeping this sheet true (Issue #118)

The code is the source of truth. This command lists every Auto that is **not** hidden. It should print the same names as page 1 (13 today):

```bash
grep -L "^@Disabled" $(grep -rl "^@Autonomous" TeamCode/src) | xargs grep -h -A1 "^@Autonomous" | grep -o 'name = "[^"]*"'
```

If a name, the count, or `preselectTeleOp` changes, update this sheet, the Autos in `ARCHITECTURE.md` section 2, and `biobuzz-auto-strategy.md`. More about the 12 Pedro Autos (what they do, risks, quick fixes in the pits): `../src/main/java/org/firstinspires/ftc/teamcode/auto/testing/README.md`. The TeleOp buttons are in `biobuzz-teleop-controls.md`.
