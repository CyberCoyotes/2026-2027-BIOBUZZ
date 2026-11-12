# Laptop checks

Small programs that run our **real** robot code on a laptop, with no robot. They are here because
our notes keep saying "checked on a laptop (41 scenarios)", but those test programs were never saved,
so nobody could run them again after changing the code. These can be run again, any time:

```bash
cd TeamCode/laptop-checks
./run-laptop-checks.sh
```

It needs `java` (17 or newer), `curl` and `unzip`. The first run downloads the real FTC SDK 12.0.0 jars
from Maven Central (about 8 MB, saved in `~/.cache/biobuzz-ftc-sdk-12.0.0`). After that it works
offline and takes a few seconds. It ends with `ALL LAPTOP CHECKS PASSED` or `SOMETHING FAILED`.

## What it does

1. **Compiles our team code against the real SDK jars.** This alone catches typos, wrong method
   names, and signs of a bad merge, the same way Android Studio would (minus Pedro Pathing: the three
   files that need Pedro are skipped).
2. **Runs the checks in `checks/`.** Each one builds the real class with fake hardware (a fake motor
   that remembers the last power it was given, a fake IMU, a made-up battery voltage) and asks it
   questions with known answers.

| Check | What it proves |
|---|---|
| `PowerMonitorDipCheck` | The battery-dip detector counts one clean dip, treats chatter around the threshold as ONE dip, counts two separate dips, ignores a healthy battery, and does not treat "the hub did not answer" as a recovery. |
| `DriveAccelerationCheck` | `DriveSubsystem` limits how fast a wheel's power rises (a real-time 0.2 s ramp), never limits stopping or slowing down, reverses through zero, and `getTurnPower()` follows the real spin. |
| `ImuSubsystemCheck` | `ImuSubsystem` reports yaw as the heading, passes pitch and roll through, resets to zero at once, and survives a missing or unstarted IMU. |

## What it does NOT do

- It is **not a robot test**. A fake motor cannot tell you the wheels are wired the right way round, or
  that the Pinpoint cable is tight, or that the IMU's orientation settings match how the hub is bolted
  down. Anything that says `[CONFIRM]` still needs the robot.
- It does not run the whole `BiobuzzTeleOp.loop()` (that needs fake versions of every subsystem).
- Gradle ignores this folder. Nothing here goes on the robot.

## Adding a check

Copy `checks/ImuSubsystemCheck.java` and change it. The pattern: make the real object WITHOUT its
constructor (the constructors need a `HardwareMap`, which only exists on a robot), put fake hardware
inside it, call its methods, and `check(...)` the answers. Then add the new class name to the `for`
loop at the bottom of `run-laptop-checks.sh`. (These files use reflection and lambdas to build the
fakes. That is fine here because this is test code, not robot code.)

`shadow/android/util/Log.java` is a do-nothing `Log` that lets `RobotLog` run on a laptop (the real
Android `Log` does not exist there).
