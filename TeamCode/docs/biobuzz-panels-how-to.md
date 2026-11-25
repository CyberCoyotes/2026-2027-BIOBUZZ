# Panels how-to (live graphs for the firing sequence)

*Written for issue #122 ("Firing sequence is requiring too much manual override"). Panels is **practice only**: it needs a laptop on the robot's wifi, so it cannot be used in a match. The volley log (below) works without it.*

**How sure are we?** The "What we know" list comes from reading the Panels library files (version 1.0.19); the Panels documentation website could not be reached when this was written. **First test on a robot, 2026-11-25: it works.** The page opens at `http://192.168.43.1:8001`, and the Graph widget draws our numbers. What is still unchecked is in the last list.

## What we know

- Panels (`com.bylazar:fullpanels:1.0.19`) runs a small web server inside the Robot Controller app. You look at it in a browser on a laptop.
- Our code sends numbers with `FiringMonitor` (the only file that uses Panels). Panels shows each one as a line `name: value`, and its **Graph** widget draws the ones that are plain numbers.
- Panels sends new data to the browser at most about **13 times a second**, so a blip shorter than about 75 ms can be missed.
- The Panels web files mention **port 8001**.
- Graphs are **off** in the code by default (`SHOW_PANELS_FIRING_GRAPHS = false` at the top of `BiobuzzTeleOp.java`). **This was the cause of "Panels is connected but there is no data" in the first test.** The Driver Station may show "Panels is currently enabled": that message comes from Panels itself (it is not in our code) and does **not** mean our switch is on. Check the constant in the code you deployed.

## Turn it on (practice)

1. In `BiobuzzTeleOp.java`, set `SHOW_PANELS_FIRING_GRAPHS = true`. Deploy.
2. Connect the laptop to the robot's wifi (the Control Hub's network).
3. Open a browser and try **`http://192.168.43.1:8001`** (the Control Hub's usual address, plus the port above).
4. INIT and START the TeleOp. Data only flows while the TeleOp's `loop()` runs.
5. **Before a match, set the switch back to `false`.**

## What to graph (the names are exactly as in `FiringMonitor.java`)

| Graph | Names | What it tells you |
|---|---|---|
| Flywheel speed | `Flywheel RPM`, `Flywheel target`, `Flywheel RPM (raw)` | Does it sit **below** the target (droop), **wobble** around it (noise), or **climb slowly from 0** (spin-up)? |
| Flywheel error | `Flywheel error (RPM)`, `At-speed band +`, `At-speed band -` | The flywheel counts as at speed only while the error is between the two band lines for 0.15 s. |
| The green light | `At speed (1=yes)`, `Turret ready (1=yes)`, `Ready to start (1=yes)`, `Sequence step (...)`, `Gate (...)` | Which half was still 0 when the operator reached for Y or B? |
| The camera | `Camera tx (deg)`, `Aim limit +`, `Aim limit -`, `Camera sees hive (1=yes)` | Is the turret really off, or does the camera wobble across the limit? |
| Power | `Flywheel amps`, `Battery (V)`, `Loop (ms)` | Below target while drawing a lot of current = no power left (battery, belt). Below target while drawing little = tuning. |
| The people | `Trigger held`, `Y lift held`, `Forced shot held`, `PRE-REV on`, `TABLE mode` | What was being pressed, and when. |

The text lines **Step**, **Waiting for**, **Last wait** and **Last feed** show on the telemetry page.

## The volley log (no laptop needed, always on)

Every volley writes two lines to the Robot Controller log, tagged `BIOBUZZ` (how to read it: top of `BiobuzzLog.java`):

- `VOLLEY WAIT: READY after 2.29 s. Waited on: flywheel only 1.77 s, turret only 0.00 s, both 0.53 s. ... At the pull: flywheel 0 RPM, PRE-REV off, TABLE. ...` (a wait that ends in **ABORTED** means the trigger was let go before the volley was ready.)
- `VOLLEY FEED: Fed for 1.5 s. Flywheel dips 3 (about 1 per ball). Intake paused 0.8 s. Flywheel low 1266 RPM (target 1300). Battery low 11.90 V.`

Copy these lines into the test sheet in `biobuzz-test-and-match-log.md` (section 2a).

## Checked on the first test (2026-11-25) and still to check

- [x] The build is clean with Panels in it, and the Panels page opens at **`http://192.168.43.1:8001`**.
- [x] The **Graph** widget draws our numbers. Add a Graph widget; once data arrives its **Variables** list shows our names, and **All** / **None** pick them. Names with spaces and parentheses (for example `Flywheel error (RPM)`) work, so no renaming was needed. The widget's "Time window" defaults to 15 s.
- [ ] The **Telemetry** widget ("T") shows the text lines `Step`, `Waiting for`, `Last wait` and `Last feed`. Did it show them? ______
- [ ] The **Loop (ms)** average with `SHOW_PANELS_FIRING_GRAPHS` on and off (the TeleOp's own average is under `FULL_TELEMETRY`). Is the extra cost small? ______
- [ ] The Driver Station screen looks the same as before (Panels sends its own copy and should not touch it).
- [ ] Does `Flywheel amps` read sensibly (zero when stopped, several amps while spinning)?

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
