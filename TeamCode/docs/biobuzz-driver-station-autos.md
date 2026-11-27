# BIOBUZZ Autons on the Driver Station

**Print this and keep it by the Driver Station.** **Page 1** is the list you pick from, with what each Auto does. **Page 2** is the check before a match, the Autos that are hidden, and what is still open. **Checked against the code on 2026-11-24 (Issue #118), then plain Drive Forward was hidden on 2026-11-26 (Issue #136), then `Blue D6 - Garden D1` was added on 2026-11-27 (Issue #156).**

The names below are copied letter for letter from the `name = "..."` text in each `@Autonomous` line, so what you see on the Driver Station should match them exactly.

**About the order:** we expect the Driver Station to sort the Autonomous list by **group, then name** (see the groups below), so the **Blue** Autos come first, then the **Red** Autos. The real Driver Station app has not been checked: **look at the real screen once. If the order is different, fix it here.** (Groups were changed on 2026-11-26, issue #136.)

## What you pick from: 13 Autos

Every Auto on the list has its alliance in the name, and is in group **Auto - Blue** or **Auto - Red**. There is no Auto without an alliance any more (issue #136). Every one loads **TeleOp - RC 1** after it ends (`preselectTeleOp`, set in one place: `BiobuzzTeleOp.TELEOP_NAME`).

```
 Autonomous                         (13 on the list)
 ----------------------------------------------------
 Blue - Drive Forward 24in          (group Auto - Blue)
 Blue - Drive Reverse 24in
 Blue D1 - Move Park                [#112]
 Blue D1 - Shoot Park               [#112]
 Blue D6 - Garden D1                (new, issue #156, changed after the first test, not run on a robot since)
 Blue D6 - Move Park
 Blue D6 - Shoot Park
 Red - Drive Forward 24in           (group Auto - Red)
 Red - Drive Reverse 24in
 Red C1 - Move Park
 Red C1 - Shoot Park
 Red C6 - Move Park                 [#112]
 Red C6 - Shoot Park                [#112]
```

**`[#112]` means the coach flagged it "Remove or fix heading" and has not decided yet.** Do not pick one of those for a match until #112 is settled.

### Match Autos (9): the robot follows a path with Pedro Pathing

Pick the one for **our alliance and the tile we start on**. Wrong alliance means it aims at the other hive and parks on the wrong side. Blue parks in the blue loading zone (F2), red in the red loading zone (A5).

| Driver Station name | Robot starts | Faces | What it does | Points | Time (estimate) |
|---|---|---|---|---|---|
| **Blue D6 - Move Park** | Tile D6, rear wall | down, at the hive | Drives into our loading zone and stops | LEAVE 3 + PARK 5 = **8** | about 4 s |
| **Blue D6 - Shoot Park** | Tile D6, rear wall | down, at the hive | Shoots the 4 preloaded POLLEN, then parks | **8**, plus any score from the shots | about 8 to 10 s |
| **Red C1 - Move Park** | Tile C1, audience wall | up, at the hive | Same as Blue D6, turned 180 degrees | **8** | about 4 s |
| **Red C1 - Shoot Park** | Tile C1, audience wall | up, at the hive | Same as Blue D6 - Shoot Park, turned 180 degrees | **8**, plus shots | about 8 to 10 s |
| **Blue D6 - Garden D1** (new) | Tile D6, rear wall | down, at the hive | Shoots the 4 preloaded POLLEN (faster than the other Autos), then the flywheel stops. Rotates left and runs along the rear wall, facing right with its left bumper along the wall, into our GARDEN in the top-right corner, sweeping down the line of POLLEN with the intake on. As it leaves the garden the flywheel revs to 1200 RPM. It goes down the right side of the hive to tile D1, turning left to face up, and shoots again, then parks. If it is out of time after the garden it skips the D1 shot and parks | **8**, plus any score from the two volleys | about 20 to 24 s (the 24 s is from a simulation with a camera that never locks, the 20 s is worked out; **close to the 30 s limit**) |
| **Blue D1 - Move Park** `[#112]` | Tile D1, audience wall, about half an inch from the blue flower | up, at the hive | Drives straight off the wall first, then parks | **8** | about 4 s |
| **Blue D1 - Shoot Park** `[#112]` | Tile D1 (same) | up, at the hive | Shoots 4, then parks | **8**, plus shots | about 8 to 10 s |
| **Red C6 - Move Park** `[#112]` | Tile C6, rear wall, next to the red flower | down, at the hive | Same idea as Blue D1, turned 180 degrees | **8** | about 4 s |
| **Red C6 - Shoot Park** `[#112]` | Tile C6 (same) | down, at the hive | Shoots 4, then parks | **8**, plus shots | about 8 to 10 s |

- **Default choice: `Shoot Park` from D6 (blue) or C1 (red).** If the shooter or camera is broken but driving works, pick `Move Park`.
- The time numbers are estimates from a simulation (`auto/testing/README.md`). **How many balls it takes to tip the hive is still unknown**, so a tip is a bonus, not a promise (`biobuzz-auto-strategy.md`, section 4).
- The D1 and C6 starts are tight: the robot starts about half an inch from a flower. Place it centered in the tile and square to the wall.
- **`Blue D6 - Garden D1` is new (issue #156) and has not run on a robot since it was changed after the first test** (the coach's comment on #156: shoot the 4th POLLEN faster, rotate left and run along the rear wall into the garden, rev the flywheel to 1200 RPM after leaving it). It starts the same way as `Blue D6 - Shoot Park`. The garden approach and the D1 shot spot are **starting guesses** (the `TODO`s in `BiobuzzAuto.java`: `GARDEN_WALL_GAP_INCHES`, `GARDEN_APPROACH_X_INCHES`, `GARDEN_CORNER_GAP_INCHES`, `GARDEN_PICK_SECONDS`, `GARDEN_INTAKE_ON_INCHES`, `GARDEN_FEED_SECONDS`, `D1_SHOT_X_INCHES`, `D1_SHOT_Y_INCHES`). **Run it in STEP mode on the practice field first** (press A during INIT). The shot from D1 needs the camera to see the blue hive from the audience side: if it never locks, the shot fires from the pre-aim angle at 1300 RPM. The rules allow a robot to hold **4 balls at most** (G407), so shot 1 has to empty the robot before the garden. Only blue is on the list: red would be the same Auto turned 180 degrees (garden in the bottom-left corner, D1 shot from tile C6), one small file away.

### Backup Autos (4): no Pedro, no camera, just drive 24 inches in a straight line

Use these if Pedro or the Pinpoint is broken, or the robot cannot get to a path start. They earn **LEAVE (3 points)** only, and they **do not park**. The 24 inches is where the robot should **end up**. Each one stops a few inches early because the robot rolls on after the wheels stop.

| Driver Station name | Put the robot like this | What it does | Tells the TeleOp the alliance? |
|---|---|---|---|
| **Blue - Drive Forward 24in** | Back against a wall | Drives forward 24 in | Yes, BLUE |
| **Red - Drive Forward 24in** | Back against a wall | Drives forward 24 in | Yes, RED |
| **Blue - Drive Reverse 24in** | **Intake** against a wall | Drives **backward** 24 in | Yes, BLUE |
| **Red - Drive Reverse 24in** | **Intake** against a wall | Drives **backward** 24 in | Yes, RED |

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
8. **After the Auto:** the Driver Station loads **TeleOp - RC 1**. Check the **alliance** line. Every Auto on the list hands it over (the eight Pedro Autos and the four "24in" Autos). If the Auto was only INIT'd and never STARTed, it shows BLUE.

## Not on the list (hidden with `@Disabled`)

The code is all still here. To bring one back, put `//` in front of its `@Disabled` line and redeploy.

| Name on the Driver Station | Group | Why it is hidden |
|---|---|---|
| Blue D6 - 3 Volley, Blue D1 - 3 Volley | Auto - Blue | Runs about 35 s, past the 30 s limit. Hidden 2026-11-21 |
| Red C1 - 3 Volley, Red C6 - 3 Volley | Auto - Red | Runs about 35 s, past the 30 s limit. Hidden 2026-11-21 |
| Leave and Park - Mark 1 | Auto | The Pedro "Move Park" Autos do the same job for each start. Hidden 2026-11-21 |
| Drive Forward (`AutoEmergencyLeave`) | Auto | It does not know the alliance, so the TeleOp fell back to BLUE. The coach wants only the Blue and Red `Drive Forward 24in` Autos (issue #136). Hidden 2026-11-26 |
| Auto - Shoot and Park - Alpha 1 | Testing | Test Auto, nobody is using it. Moving it onto the shared firing sequence is `[#89]` |
| Pedro Forward Back - Alpha | Testing | Pedro tuning test, nobody is using it |
| 1 Volley Park Mirror - Alpha, 3 Volley Mirror - Alpha, 2 Volley Rear Flower Mirror - Alpha, 2 Volley Wall Flower Mirror - Alpha | Testing | Built for our practice-field layout. **Never run these in a match** |

## Where the files are (moved 2026-11-24)

Under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:

- **`auto/`**: the four `24in` Autos, and `Move Park` and `Shoot Park` from **Blue D6** and **Red C1**.
- **`auto/testing/`**: `BiobuzzAuto` (the engine the Pedro Autos share: **do not delete this folder**), the **Blue D1** and **Red C6** Autos, **Blue D6 - Garden D1** (`AutoBlueD6GardenD1`), and the hidden Autos: the 3 Volley ones, `Leave and Park`, and plain **Drive Forward** (`AutoEmergencyLeave`).
- **A folder named `testing` does not hide an Auto.** Only `@Disabled` does. Today **Blue D1, Red C6 and Blue D6 - Garden D1 are in `auto/testing/` and are still on the Driver Station list.**

## Open items that change this sheet

- **`[#112]` Remove the broken autons.** The coach flagged the four D1 and C6 Autos "Remove or fix heading". The D6 and C1 Autos and plain **Drive Forward** were on the same review list with no problem written next to them. The note on Drive Forward ("make a blue and red") was answered by the four "24in" Autos (#114), and plain Drive Forward was hidden on 2026-11-26 (issue #136). When #112 is closed, take the markers out of this sheet and hide or remove the Autos it names.
- **`[#115]` Backward distances for each side** are still being validated.
- **Robot testing of the match Autos:** Pedro was tuned on 2026-11-17 (`FORESIGHT_TUNED` is `true` in `pedro/Constants.java`, and #57 was closed on 2026-11-18). The Autos README (written 2026-11-20) says none of the 8 has run on a real field, and no later result is written down. `Blue D6 - Garden D1` (2026-11-27) has not run on a robot either: it was compiled and run in a simulation (a fake robot and clock), and its paths were checked with a geometry script, so its **timing and the garden pick are estimates**. **Run each one you might use in STEP mode on the event's practice field before a match.** If there is no chance to test, use **Shoot Park** or **Move Park** from D6 or C1, or a **24in** backup.

## Keeping this sheet true (Issue #118)

The code is the source of truth. This command lists every Auto that is **not** hidden. It should print the same names as page 1 (13 today):

```bash
grep -L "^@Disabled" $(grep -rl "^@Autonomous" TeamCode/src) | xargs grep -h -A1 "^@Autonomous" | grep -o 'name = "[^"]*"'
```

If a name, the count, or `preselectTeleOp` changes, update this sheet, the Autos in `ARCHITECTURE.md` section 2, and `biobuzz-auto-strategy.md`. More about the 13 Pedro Autos (what they do, risks, quick fixes in the pits): `../src/main/java/org/firstinspires/ftc/teamcode/auto/testing/README.md`. The TeleOp buttons are in `biobuzz-teleop-controls.md`.
