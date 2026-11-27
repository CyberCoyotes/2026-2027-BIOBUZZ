# Panels how-to (live graphs from every OpMode)

*Started for issue #122 (the firing sequence). Since 2026-11-26 the TeleOp, every test OpMode and every Auto can send graph numbers (issue #67), and the on/off switch is one master switch (the re-check before Dec 10 is `[#138]`). Panels is for **practice**: it needs a laptop on the robot's wifi, so it cannot be used in a match. The volley log (below) works without it.*

**How sure are we?** The "What we know" list comes from reading the Panels library files (version 1.0.19; its documentation website could not be reached when this was written). **First test on a robot, 2026-11-25: it works.** The page opens at `http://192.168.43.1:8001`, and the Graph widget draws our numbers. **The 2026-11-26 changes (the master switch and all the new OpModes) compile on a laptop but have not been built with Gradle or run on a robot yet.** What is still unchecked is in the last list.

## What we know

- Panels (`com.bylazar:fullpanels:1.0.19`) runs a small web server inside the Robot Controller app. You look at it in a browser on a laptop.
- Our code sends numbers through **one class, `utils/PanelsGraphs.java`**, the only file that uses Panels. Panels shows each one as a line `name: value`, and its **Graph** widget draws the ones that are plain numbers.
- Panels sends new data to the browser at most every **75 ms** (about 13 times a second), so a blip shorter than about 75 ms can be missed. `PanelsGraphs` asks Panels before it does any work, and skips the loops Panels would throw away, so the extra loop time is small.
- The Panels web files mention **port 8001**.
- **The master switch is ON by default.** It is `PANELS_ENABLED = true` at the top of `PanelsGraphs.java`, and it turns **every** OpMode on or off together. (The old `SHOW_PANELS_FIRING_GRAPHS` in `BiobuzzTeleOp` is gone. "Panels is connected but there is no data" in the first test was that old switch being false.) **If you see no data now:** check that `PANELS_ENABLED` is `true` in the code you deployed, and that the OpMode is **running** (INIT sends nothing: data only flows while `loop()` runs). The Driver Station may show "Panels is currently enabled": that message comes from Panels itself and does **not** say anything about our switch.

## Use it (practice)

1. Deploy any OpMode. (`PANELS_ENABLED` is already `true`.)
2. Connect the laptop to the robot's wifi (the Control Hub's network).
3. Open a browser and go to **`http://192.168.43.1:8001`**.
4. INIT and START the OpMode. Add a **Graph** widget, then pick the names you want in its **Variables** list (**All** / **None** pick them all).

## Turn it off (for example for competition)

Change `PANELS_ENABLED` to `false` in `utils/PanelsGraphs.java` and redeploy. That one word turns Panels off for the TeleOp, all the Autos and all the test OpModes, and every graph call then does nothing and costs nothing. **Whether to do that before competition is the open question `[#138]` (due Dec 10).** The old rule "set it back to false before a match" is replaced by that question: the coach decides after the loop-time checks in the issue.

## What each OpMode graphs

The same thing has the **same name** in every OpMode, so one Panels layout works for all of them. Units are in the names, and yes/no answers are 1 (yes) and 0 (no).

| OpMode | What it sends |
|---|---|
| `TeleOp - Alpha 15` | Everything in the table below for the firing sequence, **plus** `Drive forward input`, `Drive strafe input`, `Drive turn input`, the turret (`Turret angle (deg)`, `Turret target (deg)`, `Turret on target (1=yes)`, `Turret spin push (power)`), the intake (`Intake motor power`, `Intake speed (ticks/s)`), the Pinpoint (`Pinpoint X (in)`, `Pinpoint Y (in)`, `Pinpoint heading (deg)`, speeds) and the IMU (`IMU heading (deg)`, pitch, roll) |
| The 12 Pedro Autos (`Blue D6 - Move Park` and the rest, all run by `BiobuzzAuto`) | `Pose X (in)`, `Pose Y (in)`, `Pose heading (deg)`, `Distance to end of path (in)` (only while Pedro is following a path), `Auto step number` and `Auto step`, `Shot number`, `Run time (s)`, `Time in this step (s)`. Routines that shoot also send the flywheel, turret, camera, gate and intake numbers and `Flywheel ready (1=yes)`. **No flywheel amps in Autos** (asking the hub costs loop time) |
| `Drive Forward` and the four `Drive Forward / Reverse 24in` Autos | `Distance driven (in)` against `Distance target (in)`, `Drive power asked (0 to 1)`, `Time driving (s)`, `Auto step (0 driving, 1 done)`, `Driving by Pinpoint distance (1=yes, 0=timed backup)` and the Pinpoint numbers |
| `Firing Sequence - Alpha` | The shooter, turret, camera, gate and intake numbers, `At speed (1=yes)`, `Sequence step number` and `Sequence step`, `Intake wanted by sequence (1=yes)` |
| The other test OpModes (hidden until you bring them back) | Intake: motor power, speed, `Intake dial power`. Shooter Power / RPM / Ratio: flywheel and hood wheel against target, error, bands, `Flywheel amps`, gate, intake. Shot Table: the shooter, the camera and `Table says flywheel RPM`. Turret: angle against target and `Turret encoder (ticks)`. Vision: camera and `Camera tag count`. Drive: the stick inputs and the Pinpoint. Flower Picker: `Flower left servo target`, `Flower right servo target`, `Flower picker up (1=yes)`. IMU: IMU, Pinpoint and `Battery (V)`. Pinpoint: the Pinpoint numbers, pod ticks and `Pinpoint update rate (Hz)` |
| `Pedro Forward Back - Alpha` | `Pose X (in)`, `Pose Y (in)`, `Pose heading (deg)`, `Distance to end of path (in)`, `Auto step number` and `Auto step` |
| Every OpMode above | `Loop (ms)`: how long one trip through `loop()` took (added by `PanelsGraphs`; the first loop is left out) |

**Not wired** (hidden and replaced by other Autos, coach's decision 2026-11-26): the four Mirror Alphas, `ShootAndParkAuto` and `AutoLeaveAndPark`. **Not graphed:** the wheel powers (they are private inside `DriveSubsystem`), so the sticks are graphed instead.

## What to graph for the firing sequence (the names are exactly as in `PanelsGraphs.java` and `FiringMonitor.java`)

| Graph | Names | What it tells you |
|---|---|---|
| Flywheel speed | `Flywheel RPM`, `Flywheel target`, `Flywheel RPM (raw)` | Does it sit **below** the target (droop), **wobble** around it (noise), or **climb slowly from 0** (spin-up)? |
| Flywheel error | `Flywheel error (RPM)`, `At-speed band +`, `At-speed band -` | The flywheel counts as at speed only while the error is between the two band lines for 0.15 s. |
| The green light | `At speed (1=yes)`, `Turret ready (1=yes)`, `Ready to start (1=yes)`, `Sequence step (...)`, `Gate (...)` | Which half was still 0 when the operator reached for Y or B? |
| The camera | `Camera tx (deg)`, `Camera ty (deg)`, `Aim limit +`, `Aim limit -`, `Camera sees hive (1=yes)`, `Camera result age (ms)` | Is the turret really off, or does the camera wobble across the limit? |
| Power | `Flywheel amps`, `Battery (V)`, `Loop (ms)` | Below target while drawing a lot of current = no power left (battery, belt). Below target while drawing little = tuning. |
| The people | `Trigger held`, `Y lift held`, `Forced shot held`, `PRE-REV on`, `TABLE mode` | What was being pressed, and when. |

The text lines **Step**, **Waiting for**, **Last wait** and **Last feed** show on the Telemetry widget ("T").

## Adding a graph to an OpMode (for programmers)

Every OpMode follows the same three steps in `loop()`. The full rules are at the top of `PanelsGraphs.java`.

1. `panelsGraphs = new PanelsGraphs();` once in `init()`.
2. `panelsGraphs.beginLoop();` as the **first** line of `loop()`.
3. Add numbers, then `panelsGraphs.send();` as the **last** line of `loop()`. **Do not `return` early in between** (a skipped `send()` leaves stale lines that go out twice in the next frame).

Use a ready-made set when there is one (`addShooter`, `addTurret`, `addVision`, `addPinpoint`, `addImu`, `addIntake`, `addGate`, `addFlowerPicker`, `addDriveInputs`, `addPose`). For anything else use `add("Name (units)", number)`, `addFlag("Name (1=yes)", yesNo)` or `addText("Name", words)`. **Plain numbers only: units go in the name, never in the value.**

## The volley log (no laptop needed, always on)

Every volley writes two lines to the Robot Controller log, tagged `BIOBUZZ` (how to read it: top of `BiobuzzLog.java`):

- `VOLLEY WAIT: READY after 2.29 s. Waited on: flywheel only 1.77 s, turret only 0.00 s, both 0.53 s. ... At the pull: flywheel 0 RPM, PRE-REV off, TABLE. ...` (a wait that ends in **ABORTED** means the trigger was let go before the volley was ready.)
- `VOLLEY FEED: Fed for 1.5 s. Flywheel dips 3 (about 1 per ball). Intake paused 0.8 s. Flywheel low 1266 RPM (target 1300). Battery low 11.90 V.`

Copy these lines into the test sheet in `biobuzz-test-and-match-log.md` (section 2a).

## Checked, and still to check

- [x] The build is clean with Panels in it, and the Panels page opens at **`http://192.168.43.1:8001`** (2026-11-25).
- [x] The **Graph** widget draws our numbers (2026-11-25). Add a Graph widget; once data arrives its **Variables** list shows our names, and **All** / **None** pick them. Names with spaces and parentheses (for example `Flywheel error (RPM)`) work, so no renaming was needed. The widget's "Time window" defaults to 15 s.
- [x] **On a laptop (2026-11-26):** every file that does not need Pedro compiles against the real FTC SDK 12.0.0 and Panels jars, and `PanelsGraphs` ran against the real Panels `TelemetryManager` in a throwaway test (a fast second loop sends nothing, one frame per 75 ms, yes/no as 1 and 0, switch off does nothing). The two Pedro files compiled against small stand-ins.
- [ ] A **Gradle build** of the whole project after the 2026-11-26 changes (the laptop check could not use Gradle or the Pedro library). Result: ______
- [ ] **Deploy and open Panels** with the TeleOp, with one Pedro Auto and with one test OpMode. Do the new names appear in the Graph widget (for example `Pose X (in)`, `Turret angle (deg)`, `Drive forward input`)? ______
- [ ] The **Telemetry** widget ("T") shows the text lines `Step`, `Waiting for`, `Last wait` and `Last feed`. Did it show them? ______
- [ ] **Loop time (`[#138]`).** The **Loop (ms)** average with the master switch on and off, in the TeleOp and in one Pedro Auto (`Blue D6 - Shoot Park`). Is the extra cost small? ______ (The TeleOp's own average is under `FULL_TELEMETRY`; `Loop (ms)` only shows in Panels, so compare with the switch off using the TeleOp's average.)
- [ ] The Driver Station screen looks the same as before (Panels sends its own copy and should not touch it).
- [ ] Does `Flywheel amps` read sensibly (zero when stopped, several amps while spinning)?
- [ ] A match-style run with **no laptop connected** works exactly as before (the normal case in a match).

## Deploy over wifi (ADB) with the same laptop connection

*Confirmed working by the coach on 2026-11-25: connected with `adb connect 192.168.43.1:5555` and deployed from Android Studio over the robot's wifi.* The Control Hub accepts ADB over its wifi (port 5555), so one connection to the robot's network serves **both** Panels and Android Studio. No USB cable is needed.

1. Deploy once with the USB cable, so the project is built and Gradle has downloaded everything (the robot's wifi has no internet).
2. Join the Control Hub's wifi network from the laptop.
3. Open Android Studio's **Terminal** and run (use the full path to `adb.exe` in the Android SDK's `platform-tools` folder if `adb` is not found):

```bash
adb connect 192.168.43.1:5555
adb devices
```

4. `adb devices` should list `192.168.43.1:5555   device`. Pick that device (the Control Hub) in Android Studio's device menu and press Run.
5. If it says `offline` or refuses: `adb kill-server`, then `adb connect 192.168.43.1:5555` again. If Gradle complains it cannot reach a repository, turn on **Offline Mode** in Gradle's settings.
6. This works for a **Control Hub**. A Robot Controller phone first needs `adb tcpip 5555` once over USB.
