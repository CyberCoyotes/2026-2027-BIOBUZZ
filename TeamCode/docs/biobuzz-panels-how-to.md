# Panels how-to (live graphs for the firing sequence)

*Written for issue #122 ("Firing sequence is requiring too much manual override"). Panels is **practice only**: it needs a laptop on the robot's wifi, so it cannot be used in a match. The volley log (below) works without it.*

**How sure are we?** The "What we know" list comes from reading the Panels library files (version 1.0.19). The Panels documentation website could not be reached when this was written, so the "Check on the first test" list is **not confirmed yet**. Fill in the real answers after the first test and delete the guesses.

## What we know

- Panels (`com.bylazar:fullpanels:1.0.19`) runs a small web server inside the Robot Controller app. You look at it in a browser on a laptop.
- Our code sends numbers with `FiringMonitor` (the only file that uses Panels). Panels shows each one as a line `name: value`, and its **Graph** widget draws the ones that are plain numbers.
- Panels sends new data to the browser at most about **13 times a second**, so a blip shorter than about 75 ms can be missed.
- The Panels web files mention **port 8001**.
- Graphs are **off** in the code by default (`SHOW_PANELS_FIRING_GRAPHS = false` at the top of `BiobuzzTeleOp.java`).

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

## Check on the first test (not confirmed yet)

- [ ] The build is clean with Panels in it, and the Panels page opens at the address above. Write the real address here: ______
- [ ] How the **Graph** widget picks what to draw (the names above should show up in a list). Write the steps here: ______
- [ ] The **Loop (ms)** average with `SHOW_PANELS_FIRING_GRAPHS` on and off (the TeleOp's own average is under `FULL_TELEMETRY`). Is the extra cost small? ______
- [ ] The Driver Station screen looks the same as before (Panels sends its own copy and should not touch it).
- [ ] Does `Flywheel amps` read sensibly (zero when stopped, several amps while spinning)?
