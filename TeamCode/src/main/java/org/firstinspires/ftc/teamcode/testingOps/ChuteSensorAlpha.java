package org.firstinspires.ftc.teamcode.testingOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ChuteSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.PanelsGraphs;

/*
 * Alpha-Chute Sensor Testing (issue #144, Phases 1 and 2)
 * An iterative OpMode that shows what the REV Color Sensor V3 in the chute (named chute_sensor)
 * reports: the distance and the color. Phase 2 adds the "Pollen at top" answer and a count of how
 * often it changes. It uses gamepad1 only.
 *
 *   A            -> RESET the min and max distance (so you can catch a ball rolling past) and the
 *                   count of answer changes
 *   X            -> color reading ON / OFF (to see what the color read costs)
 *   Right Bumper -> INTAKE (hold) so you can run a ball past the sensor
 *   Left Bumper  -> REVERSE (hold). It beats Right Bumper, so clearing a jam always works.
 *
 * WHAT THIS IS FOR
 *   We want the robot to know when a Pollen is at the top of the chute, so the last ball can be
 *   lifted into the shooter by the sensor instead of by the operator holding Y. Before we write
 *   any "there is a Pollen" rule (Phase 2), we have to SEE what the sensor reads. This OpMode only
 *   shows numbers. It decides nothing itself, and it does not move the gate. The "Pollen at top"
 *   answer comes from ChuteSensorSubsystem.hasPollenAtTop() (how it decides is explained at the top
 *   of that file). The numbers it uses are ENTER 3.0 cm, EXIT 3.8 cm and a 0.10 s wait. They are
 *   starting values from the coach's hand readings, and THIS OpMode is how we check them.
 *
 * WHY THE "ANSWER CHANGES" COUNT (Phase 2)
 *   The answer should flip once when a Pollen arrives and once when it leaves. If the count climbs
 *   while a ball just SITS there (or while the sensor sees an empty chute), the numbers are too
 *   tight: the reading is wandering across them. Press A before each try, so the count starts at 0.
 *   The Panels graph (see biobuzz-panels-how-to.md) draws the distance with the ENTER and EXIT
 *   lines and the answer, so you can see where it crossed.
 *
 * WHY THE MIN AND MAX
 *   A ball rolling past the sensor is gone in a few loops, too fast to read off the screen. Press A,
 *   run the ball past, and the screen keeps the closest and farthest readings it saw. The
 *   Panels graph (see biobuzz-panels-how-to.md) shows the whole trace. Only real readings count:
 *   "out of range" (nothing close) is shown separately, so it cannot spoil the max.
 *
 * THE INTAKE BUMPERS
 *   They use IntakeSubsystem at its normal power. The intake motor also runs the transfer
 *   grippers in the chute, so a ball in the chute is carried up. Keep hands and cables clear.
 *
 * TEST ORDER (write each result in TeamCode/docs/biobuzz-test-and-match-log.md, table 2b)
 *   1. Check the screen says "Sensor: FOUND". If it says NOT FOUND, add a REV Color Sensor V3 named
 *      chute_sensor to the robot config (see below), and check the cable before the code.
 *   2. EMPTY chute. Press A, wait 10 seconds. Write the distance (or "OUT OF RANGE") and the color.
 *      Is it steady? Is the sensor seeing the far wall, or nothing?
 *   3. One Pollen held by hand at the top of the chute, below the turret. Press A before each try.
 *      Try three ways: a HOLE facing the sensor, a solid side facing it, and slowly turning the
 *      ball. Write the min and max distance, and the hue and light level. The Pollen has holes, so
 *      we need to know if they fool the sensor.
 *   4. THE STALL SPOT. Run one real volley in the TeleOp and watch where the LAST ball stops when
 *      nothing is behind it. Put a Pollen in that same spot by hand. Does the sensor see it? If the
 *      stalled ball is NOT in the sensor's view, tell the coach: the plan for Phase 3 changes.
 *   5. FEEDING PAST. Press A. Load balls at the bottom and HOLD the Right Bumper so they ride up the
 *      chute. Write the min and max, and look at the Panels graph. Do the balls show up as
 *      separate dips, or do they run together? Try 1 ball, then 3.
 *   6. LIGHT. Repeat steps 2, 3 and 5 under the lights we will really use (the school gym, then the
 *      field or pit lights). The distance sensor is sensitive to room light.
 *   7. COLOR COST. Press X to turn the color reading OFF and look at "Sensor read time" and "Loop".
 *      Then turn it ON again. Write both. A slow loop slows everything on the robot.
 *
 * PHASE 2 TEST ORDER (write each result in the test log, table 2c)
 *   8. EMPTY chute, press A, wait 10 seconds. "Pollen at top" should say no, and the changes count
 *      should stay 0. If it says YES, the reading is at or below ENTER with nothing there.
 *   9. A Pollen held STAGED by hand (solid side, then a HOLE facing the sensor, then slowly
 *      turning), 10 seconds each, pressing A before each. "Pollen at top" should say YES and the
 *      count should be 1 (it flipped once, on the way in). A higher count means it flickered:
 *      write which side of the ball did it.
 *  10. Take the Pollen away. "Pollen at top" should go back to no (count 2). If it STAYS YES, the
 *      empty chute reads below EXIT: try a smaller EXIT (such as 3.5) in ChuteSensorSubsystem.
 *  11. THE GAP. Slide a Pollen slowly toward the sensor, then away. Watch the band line go CLEAR,
 *      BETWEEN, STAGED, then back. The answer should flip only at the two ends, not in BETWEEN.
 *  12. FEEDING PAST. Press A, load balls at the bottom and HOLD the Right Bumper. Does "Pollen at
 *      top" blip YES as a ball rides past? How many times, and for how long (see the Panels
 *      graph)? If it blips, the wait (POLLEN_AT_TOP_DEBOUNCE_SECONDS) needs to be longer.
 *  13. THE STALL SPOT. Put a Pollen where the LAST ball stops after a volley. It should say YES
 *      and stay YES.
 *  14. LIGHT. Repeat 8, 9 and 12 under the lights we will really use.
 *
 * On a PS5 controller: the bumpers are L1 and R1, A is Cross and X is Square.
 *
 * Robot configuration (on the Driver Station):
 *   I2C device name: chute_sensor     Type: REV Color Sensor V3 (Control Hub I2C bus 1 on 22091)
 *   Motor name: intake                Motor type: goBILDA 5202/3/4 series
 *   Only the 22091 robot has the sensor. On another robot this OpMode runs but says NOT FOUND
 *   (the intake still needs its motor).
 *
 * HIDE IT WHEN PHASE 2 TESTING IS DONE
 *   Once the numbers are in the test log, put @Disabled above the @TeleOp line to keep the Driver
 *   Station list short (never delete the file). The sensor class stays.
 */
@TeleOp(name = "Chute Sensor - Alpha", group = "Testing")
public class ChuteSensorAlpha extends OpMode {

    // true = the control map stays on the Driver Station screen while the OpMode RUNS (at the
    // bottom, under the live data). false = it is only shown before START.
    private static final boolean SHOW_CONTROLS_WHILE_RUNNING = true;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ChuteSensorSubsystem chuteSensorSubsystem;
    private IntakeSubsystem intakeSubsystem;

    // Sends this OpMode's numbers to Panels (practice graphs). The master switch is in PanelsGraphs.
    private PanelsGraphs panelsGraphs;

    // Times one trip around loop(), to see what the sensor does to the loop speed.
    private ElapsedTime loopTimer = new ElapsedTime();
    private double loopMillis = 0.0;

    // The closest and farthest REAL distance since the last reset (press A), in centimeters.
    // haveRealReading stays false until the first in-range reading, so the first one sets both.
    private boolean haveRealReading = false;
    private double minDistanceCm = 0.0;
    private double maxDistanceCm = 0.0;

    // true = at least one "out of range" (nothing close) reading since the last reset. An empty
    // chute may read this way, which is useful to know.
    private boolean sawOutOfRange = false;

    // How many times the "Pollen at top" answer changed since the last reset (press A). It should
    // be 1 when a Pollen arrives and 2 when it leaves. More than that means the answer flickered.
    // lastPollenAtTop is the answer from the loop before, so we can see when it changes.
    private int answerChanges = 0;
    private boolean lastPollenAtTop = false;

    // The color reading switch. It starts ON.
    private boolean colorReadingOn = true;

    // What the buttons were doing LAST loop. A button press lasts many loops, but we want it to
    // count only ONCE. So we only act when it was NOT pressed last loop and IS pressed now.
    private boolean aWasPressed = false;
    private boolean xWasPressed = false;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("A = reset min / max and answer changes   X = color reading ON / OFF");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Each subsystem gets its own hardware from the hardware map. The sensor does not stop the
        // OpMode if it is missing: the screen says NOT FOUND.
        chuteSensorSubsystem = new ChuteSensorSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        // Panels graphs (the master switch is inside PanelsGraphs). It only LOOKS, it moves nothing.
        panelsGraphs = new PanelsGraphs();

        telemetry.addData("Status", "Initialized");
        chuteSensorSubsystem.addTelemetry(telemetry);
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Reading a sensor before START is fine, so you can check it is FOUND and watch it change.
        chuteSensorSubsystem.update();

        // The Driver Station clears its screen every loop, so we add everything again here.
        telemetry.addData("Status", "Ready to start!");
        chuteSensorSubsystem.addTelemetry(telemetry);
        addControlsTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        loopTimer.reset();

        // The answer may already be YES (a Pollen was sitting there during INIT). Remember it now,
        // so the first loop does not count that as a change.
        lastPollenAtTop = chuteSensorSubsystem.hasPollenAtTop();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // How long since the last loop started (the very first one is not a real loop time).
        loopMillis = loopTimer.milliseconds();
        loopTimer.reset();

        // Open this loop's Panels frame. The matching send() is the LAST line of loop().
        panelsGraphs.beginLoop();

        // 1. Read sensors.
        chuteSensorSubsystem.update();
        intakeSubsystem.update();

        // 2. Read the gamepad and decide what to do.
        boolean aPressed = gamepad1.a;
        boolean xPressed = gamepad1.x;

        // Reset the min and max only on the loop where A FIRST goes down.
        if (aPressed && !aWasPressed) {
            resetMinMax();
        }

        // Flip the color reading only on the loop where X FIRST goes down.
        if (xPressed && !xWasPressed) {
            colorReadingOn = !colorReadingOn;
        }

        // Remember the buttons for the next loop.
        aWasPressed = aPressed;
        xWasPressed = xPressed;

        // Keep the min and max up to date with what the sensor just read.
        updateMinMax();

        // Count the times the "Pollen at top" answer changed.
        countAnswerChanges();

        // 3. Tell the subsystems what to do.
        chuteSensorSubsystem.setColorReadingOn(colorReadingOn);

        // Check REVERSE first, so clearing a jam always works.
        String intakeButton;
        if (gamepad1.left_bumper) {
            intakeSubsystem.reverse();
            intakeButton = "Left Bumper (REVERSE)";
        } else if (gamepad1.right_bumper) {
            intakeSubsystem.intake();
            intakeButton = "Right Bumper (INTAKE)";
        } else {
            // No bumper held, so stop the intake.
            intakeSubsystem.stop();
            intakeButton = "None";
        }

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Intake button", intakeButton);
        chuteSensorSubsystem.addTelemetry(telemetry);
        addMinMaxTelemetry();
        telemetry.addData("Loop time", "%.1f ms", loopMillis);

        // 5. Keep the button map on the screen while running, at the bottom under the live data.
        //    (Before START the same map is shown by init() and init_loop().)
        if (SHOW_CONTROLS_WHILE_RUNNING) {
            addControlsTelemetry();
        }

        // Panels graphs: add this OpMode's numbers, then send the frame (the LAST line of loop()).
        addPanelsGraphs();
        panelsGraphs.send();
    }

    // ------------------------------------------------------------------
    // MIN AND MAX (so a ball rolling past can be caught)
    // ------------------------------------------------------------------

    // Forget the old min and max and the answer-change count, and start over.
    private void resetMinMax() {
        haveRealReading = false;
        minDistanceCm = 0.0;
        maxDistanceCm = 0.0;
        sawOutOfRange = false;
        answerChanges = 0;
    }

    // Adds one to the count every time the "Pollen at top" answer is different from last loop.
    private void countAnswerChanges() {
        boolean pollenAtTop = chuteSensorSubsystem.hasPollenAtTop();
        if (pollenAtTop != lastPollenAtTop) {
            answerChanges = answerChanges + 1;
        }
        lastPollenAtTop = pollenAtTop;
    }

    // Looks at the reading update() just saved. A real reading can move the min or the max. An "out
    // of range" reading is only noted, so the "nothing there" number never becomes the max.
    private void updateMinMax() {
        if (!chuteSensorSubsystem.isConnected()) {
            return;
        }

        if (!chuteSensorSubsystem.isDistanceInRange()) {
            sawOutOfRange = true;
            return;
        }

        double distanceCm = chuteSensorSubsystem.getDistanceCm();
        if (!haveRealReading) {
            // The first real reading since the reset sets both.
            minDistanceCm = distanceCm;
            maxDistanceCm = distanceCm;
            haveRealReading = true;
            return;
        }

        if (distanceCm < minDistanceCm) {
            minDistanceCm = distanceCm;
        }
        if (distanceCm > maxDistanceCm) {
            maxDistanceCm = distanceCm;
        }
    }

    private void addMinMaxTelemetry() {
        if (!chuteSensorSubsystem.isConnected()) {
            return;
        }

        if (haveRealReading) {
            telemetry.addData("Distance since reset (A)", "min %.1f cm   max %.1f cm", minDistanceCm, maxDistanceCm);
        } else {
            telemetry.addData("Distance since reset (A)", "no real reading yet");
        }

        String outOfRangeText = "no";
        if (sawOutOfRange) {
            outOfRangeText = "YES (nothing was close at some point)";
        }
        telemetry.addData("Out of range seen since reset?", outOfRangeText);

        // Phase 2: 1 = it flipped once on the way in, 2 = and once on the way out. More = flicker.
        telemetry.addData("Pollen-at-top answer changes since reset (A)", "%d   (1 in, 2 out. More means it flickered)",
                answerChanges);
    }

    // ------------------------------------------------------------------
    // PANELS GRAPH NUMBERS (issue [#138]). Plain numbers with the units in the names (see
    // PanelsGraphs), so Panels can graph them. It does nothing on a loop Panels will not send.
    // ------------------------------------------------------------------
    private void addPanelsGraphs() {
        if (!panelsGraphs.isFrameOpen()) {
            return;
        }
        // A robot with no sensor sends no chute numbers, so a flat "100 cm" line cannot be mistaken
        // for a real reading.
        if (chuteSensorSubsystem.isConnected()) {
            panelsGraphs.add("Chute distance (cm, 100 = out of range)", chuteSensorSubsystem.getDistanceCm());
            panelsGraphs.addFlag("Chute distance real reading (1=yes)", chuteSensorSubsystem.isDistanceInRange());
            panelsGraphs.add("Chute sensor read (ms)", chuteSensorSubsystem.getReadMillis());
            // Phase 2: the answer, and the two lines it uses, so the distance trace can be read against them.
            panelsGraphs.addFlag("Chute pollen at top (1=yes)", chuteSensorSubsystem.hasPollenAtTop());
            panelsGraphs.add("Chute ENTER (cm)", chuteSensorSubsystem.getPollenAtTopEnterCm());
            panelsGraphs.add("Chute EXIT (cm)", chuteSensorSubsystem.getPollenAtTopExitCm());
            if (chuteSensorSubsystem.isColorReadingOn()) {
                panelsGraphs.add("Chute red", chuteSensorSubsystem.getRed());
                panelsGraphs.add("Chute green", chuteSensorSubsystem.getGreen());
                panelsGraphs.add("Chute blue", chuteSensorSubsystem.getBlue());
                panelsGraphs.add("Chute light level", chuteSensorSubsystem.getLightLevel());
                panelsGraphs.add("Chute hue (deg)", chuteSensorSubsystem.getHueDegrees());
            }
        }
        panelsGraphs.addIntake(intakeSubsystem);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motor stopped.
        intakeSubsystem.stop();
    }
}
