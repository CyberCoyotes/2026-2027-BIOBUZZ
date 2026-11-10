package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HiveBearingMemory;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

/*
 * Drive Alpha
 * An iterative OpMode that drives the robot with the DriveSubsystem using gamepad1.
 * It can drive two ways:
 *   FIELD-CENTRIC  (it starts here): "forward" means AWAY FROM YOU, even after the robot turns.
 *                   It needs the Pinpoint, which tells it how far the robot has turned.
 *   ROBOT-CENTRIC: "forward" means the way the robot is facing.
 * (This is the test for DriveSubsystem.driveFieldCentric, issue #54. The TeleOp gets the same
 *  idea later, with the operator's MODE button.)
 *
 * DRIVING
 *   Left Stick  -> drive forward / backward and slide (strafe) left / right
 *   Right Stick -> turn (push right to turn right)
 *   HOLD the Right Trigger -> SLOW MODE for careful lining up
 *   Back        -> switch FIELD-CENTRIC <-> ROBOT-CENTRIC (one switch per press). If field-centric
 *                  was switched off by the safety rule below, Back turns it back ON.
 *   D-pad Left  -> RE-AIM: "the way the robot faces right now is AWAY FROM ME". It does not touch
 *                  the Pinpoint, it only changes which way "forward" means. It also turns
 *                  field-centric back on after the safety rule switched it off.
 *
 * HOW FIELD-CENTRIC KNOWS WHICH WAY IS "AWAY FROM YOU"
 *   The Pinpoint's heading is ZERO at the moment you press INIT. So: stand where you will drive
 *   from, put the robot on the floor FACING AWAY FROM YOU, and press INIT. The robot must be STILL
 *   while the Pinpoint calibrates. If you turn the robot by hand later and want "forward" to mean
 *   the way it faces now, press D-pad Left.
 *
 * THE SAFETY RULE (a bad heading must not make the robot drive the wrong way)
 *   If the Pinpoint is not READY for MORE than PINPOINT_GLITCH_TOLERANCE_SECONDS, or its heading
 *   jumps by more than MAX_PINPOINT_HEADING_JUMP_DEGREES in one loop (it restarted), field-centric
 *   switches ITSELF OFF and the robot drives ROBOT-CENTRIC. The screen says why. It does NOT switch
 *   itself back on, because a restarted Pinpoint has a new zero. Press Back or D-pad Left to turn
 *   it on again.
 *   GLITCH DEBOUNCE (the same as BiobuzzTeleOp): a gap SHORTER than PINPOINT_GLITCH_TOLERANCE_SECONDS
 *   does not switch it off. The drive keeps the LAST good heading, and when the Pinpoint is READY
 *   again its heading must be within RESUME_MAX_HEADING_CHANGE_DEGREES of the last good one.
 *   THE DROP LOG: the "Pinpoint drops" and "Last drop" lines on the screen (from PinpointSubsystem)
 *   say which status the Pinpoint went through, how long it lasted, and how far the heading moved.
 *   If it keeps dropping, copy those two lines down: they say WHY (a pod cable, the I2C cable, or
 *   a restart).
 *
 * FIELD-CENTRIC TEST (do these in order, on the floor with lots of space):
 *   1. Robot facing AWAY from you. INIT, wait for "Pinpoint status: READY", press START.
 *   2. Push the left stick straight ahead. The robot goes AWAY from you.
 *   3. Turn the robot about 90 degrees with the RIGHT stick, then push the left stick straight
 *      ahead again. It must STILL go away from you (it slides sideways, relative to itself).
 *   4. Hold the left stick straight ahead and spin the robot a FULL 360 degrees with the right
 *      stick. It must keep moving in the SAME direction on the floor the whole time.
 *   5. Turn the robot by hand, press D-pad Left. Now "forward" is the way it faces.
 *   6. Press Back. The screen says ROBOT-CENTRIC, and the stick goes the way the robot faces.
 *      Press Back again to get field-centric back.
 *   7. SAFETY: unplug the Pinpoint (or run with it missing from the configuration). At START the
 *      robot drives ROBOT-CENTRIC and the screen says the Pinpoint is not READY.
 *
 * WHEEL TEST (for checking motor directions, with the robot ON BLOCKS so the wheels are in the air)
 *   HOLD a button to spin ONE wheel at WHEEL_TEST_POWER. It must push the robot FORWARD.
 *     X (Square)   -> left front
 *     A (Cross)    -> left rear
 *     Y (Triangle) -> right front
 *     B (Circle)   -> right rear
 *   The wheel test beats the sticks. If a wheel spins backward, flip THAT motor's direction in
 *   DriveSubsystem.
 *
 * SAFE FIRST TEST (do these in order):
 *   1. Robot on blocks, wheels in the air. Do the WHEEL TEST above for all four wheels.
 *   2. Robot on the floor with lots of space. Push the left stick FORWARD a little.
 *      The robot must go forward in a straight line.
 *   3. Push the left stick LEFT. The robot must slide left without turning.
 *   4. Push the right stick LEFT. The robot must spin counterclockwise (viewed from above).
 *   5. Try moves together (forward while turning). Then try slow mode.
 *   (The robot starts facing away from you, so field-centric drives the same as robot-centric
 *    until the robot turns. Press Back first if you want pure robot-centric for these steps.)
 *
 * WHY THE STICK SIGNS ARE FLIPPED
 *   The subsystem uses the same convention as Pedro Pathing, Pinpoint and the turret:
 *   forward +, strafe LEFT +, turn COUNTERCLOCKWISE +. The controller's sticks are different:
 *   pushing a stick forward gives a NEGATIVE number, and pushing it right gives a POSITIVE
 *   number. So we flip the sign of all three.
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station):
 *   Motor names: left_front, left_rear, right_front, right_rear
 *   I2C device name: pinpoint (the goBILDA Pinpoint). Without it this OpMode still drives, but
 *   only ROBOT-CENTRIC.
 *
 * HIDDEN FROM THE DRIVER STATION LIST
 *   The drive wheels are well tested, so this OpMode is normally @Disabled to keep the list short.
 *   The code is all still here. It is SHOWN right now (the @Disabled line below has // in front
 *   of it) so the field-centric drive can be tested. When the testing is done, take the // off
 *   and redeploy to hide it again.
 */
// @Disabled   (turned off for the field-centric test, issue #54. Remove the // to hide it again.)
@TeleOp(name = "Drive Alpha", group = "Testing")
public class DriveAlpha extends OpMode {

    // The sticks do not always rest at exactly zero. Anything smaller than this is ignored,
    // so the robot does not creep when you let go.
    private static final double STICK_DEADBAND = 0.05;

    // The trigger gives a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // Power for the one-wheel direction test. Keep it low and safe.
    private static final double WHEEL_TEST_POWER = 0.4;

    // A robot cannot turn this many degrees in one loop (about 20 milliseconds). If the Pinpoint's
    // heading jumps by more than this, it restarted or glitched, so its zero has moved and the
    // field-centric direction would be wrong. (The same number BiobuzzTeleOp uses to protect the
    // hive memory.)
    private static final double MAX_PINPOINT_HEADING_JUMP_DEGREES = 45.0;

    // A Pinpoint that is "not READY" for LESS than this many seconds is a GLITCH, not a restart:
    // field-centric stays on and the drive holds the last good heading. Longer than this, we treat
    // it as a restart and switch field-centric off. (The same numbers BiobuzzTeleOp uses.)
    private static final double PINPOINT_GLITCH_TOLERANCE_SECONDS = 0.25;

    // Coming back from a glitch, the heading must be within this many degrees of the last good one,
    // or the Pinpoint probably restarted (its heading is back at a new zero).
    private static final double RESUME_MAX_HEADING_CHANGE_DEGREES = 20.0;

    // During a glitch: true = keep driving FIELD-CENTRIC with the last good heading. false = drive
    // ROBOT-CENTRIC until the Pinpoint is READY again.
    private static final boolean HOLD_LAST_HEADING_DURING_GLITCH = true;

    // true = the control map stays on the Driver Station screen while the OpMode RUNS (at the
    // bottom, under the live data). false = it is only shown before START, like it used to be.
    private static final boolean SHOW_CONTROLS_WHILE_RUNNING = true;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveSubsystem driveSubsystem;
    private PinpointSubsystem pinpointSubsystem;

    // Does the driver WANT field-centric? (Back switches it.) It starts ON.
    private boolean fieldCentricWanted = true;

    // true = the safety rule switched field-centric off. It stays off until Back or D-pad Left.
    // The reason is shown on the screen.
    private boolean fieldCentricBlocked = false;
    private String blockedReason = "";

    // Added to the Pinpoint's heading to get the heading we drive with. It starts at 0 (the
    // Pinpoint is zero at INIT, and the robot faces away from the driver then). D-pad Left sets it
    // so the heading we drive with is 0 right now.
    private double headingOffsetDegrees = 0.0;

    // The heading we used on the last loop, and whether there is one yet, for the jump check.
    private double lastPinpointHeadingDegrees = 0.0;
    private boolean haveLastPinpointHeading = false;

    // The glitch debounce. pinpointNotReadyTimer counts the seconds since the Pinpoint stopped being
    // READY. pinpointWasReadyLastLoop lets us spot the loop where it stops, and the loop where it
    // comes back. The count and the longest glitch are for the screen.
    private ElapsedTime pinpointNotReadyTimer = new ElapsedTime();
    private boolean pinpointWasReadyLastLoop = true;
    private int pinpointGlitchCount = 0;
    private double pinpointLongestGlitchSeconds = 0.0;

    // What the buttons were doing LAST loop, so one press counts only once.
    private boolean wasBackPressed = false;
    private boolean wasReAimPressed = false;

    /*
     * True only on the loop where a button goes from "not pressed" to "pressed".
     */
    private boolean isNewPress(boolean pressedNow, boolean pressedLastLoop) {
        return pressedNow && !pressedLastLoop;
    }

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Left Stick = drive and strafe   Right Stick = turn");
        telemetry.addLine("Right Trigger = SLOW MODE");
        telemetry.addLine("Back = switch FIELD-CENTRIC / ROBOT-CENTRIC");
        telemetry.addLine("D-pad Left = RE-AIM (the way the robot faces now is AWAY from me)");
        telemetry.addLine("WHEEL TEST (robot on blocks!):");
        telemetry.addLine("  X = left front   A = left rear   Y = right front   B = right rear");
    }

    /*
     * Shows the RAW numbers from gamepad1, before any deadband or sign flip.
     * This is for finding a controller problem: with your hands OFF the controller, every stick
     * should read about 0.00, both triggers 0.00, and every button false. Anything else means
     * the controller (or how the Driver Station sees it) is the problem, not the drive code.
     */
    private void addRawGamepadTelemetry() {
        telemetry.addLine("--- RAW GAMEPAD 1 (hands off = all zeros / false) ---");
        telemetry.addData("Controller type", gamepad1.type());
        telemetry.addData("Left stick x, y", "%.2f, %.2f",
                gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Right stick x, y", "%.2f, %.2f",
                gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("Triggers L, R", "%.2f, %.2f",
                gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Face buttons", "X=%b  A=%b  Y=%b  B=%b",
                gamepad1.x, gamepad1.a, gamepad1.y, gamepad1.b);
        telemetry.addData("Back, D-pad Left", "%b, %b", gamepad1.back, gamepad1.dpad_left);
    }

    /*
     * Ignores a stick value that is too small to be on purpose.
     */
    private double applyDeadband(double stickValue) {
        if (Math.abs(stickValue) < STICK_DEADBAND) {
            return 0.0;
        }
        return stickValue;
    }

    /*
     * The safety rule. Call it once per loop, right after pinpointSubsystem.update(). It only
     * ever switches field-centric OFF. Turning it back on is up to the driver (Back or D-pad Left).
     *   - Pinpoint not READY for LONGER than PINPOINT_GLITCH_TOLERANCE_SECONDS: it is missing, still
     *     calibrating, faulted, or it restarted. A shorter gap is a GLITCH and is forgiven.
     *   - Heading jumped: it changed by more than a robot can turn in one loop, so the Pinpoint
     *     restarted or glitched. Its heading is back at some new zero, so "away from me" is wrong.
     *   - Coming back from a glitch, the heading must be within RESUME_MAX_HEADING_CHANGE_DEGREES of
     *     the last good one, or it probably restarted.
     */
    private void checkPinpointHealth() {
        if (!pinpointSubsystem.isReady()) {
            if (pinpointWasReadyLastLoop) {
                // The gap just started. Count the seconds from now.
                pinpointNotReadyTimer.reset();
                pinpointWasReadyLastLoop = false;
            }
            if (pinpointNotReadyTimer.seconds() > PINPOINT_GLITCH_TOLERANCE_SECONDS) {
                // Too long to be a glitch, so treat it as a restart.
                fieldCentricBlocked = true;
                blockedReason = String.format("the Pinpoint was not READY for more than %.2f seconds",
                        PINPOINT_GLITCH_TOLERANCE_SECONDS);
                haveLastPinpointHeading = false;
            }
            return;
        }

        // The Pinpoint is READY. Did it just come back from a gap?
        boolean justCameBack = !pinpointWasReadyLastLoop;
        double gapSeconds = pinpointNotReadyTimer.seconds();
        pinpointWasReadyLastLoop = true;

        double heading = pinpointSubsystem.getHeadingDegrees();
        if (haveLastPinpointHeading) {
            double change = Math.abs(HiveBearingMemory.wrapDegrees(heading - lastPinpointHeadingDegrees));
            if (justCameBack) {
                if (change > RESUME_MAX_HEADING_CHANGE_DEGREES) {
                    fieldCentricBlocked = true;
                    blockedReason = String.format(
                            "the Pinpoint came back from a glitch with its heading %.0f degrees off (it probably restarted)",
                            change);
                } else {
                    pinpointGlitchCount = pinpointGlitchCount + 1;
                    if (gapSeconds > pinpointLongestGlitchSeconds) {
                        pinpointLongestGlitchSeconds = gapSeconds;
                    }
                }
            } else if (change > MAX_PINPOINT_HEADING_JUMP_DEGREES) {
                fieldCentricBlocked = true;
                blockedReason = String.format("the Pinpoint heading jumped %.0f degrees", change);
            }
        }
        lastPinpointHeadingDegrees = heading;
        haveLastPinpointHeading = true;
    }

    /*
     * True while the Pinpoint is not READY but the gap is still short enough to be a glitch, and
     * we have a good heading to hold.
     */
    private boolean isPinpointInGlitch() {
        return HOLD_LAST_HEADING_DURING_GLITCH
                && !pinpointSubsystem.isReady()
                && haveLastPinpointHeading;
    }

    /*
     * The Pinpoint heading the field-centric drive uses: the live one, or during a glitch the last
     * good one.
     */
    private double getPinpointHeadingForDrive() {
        if (pinpointSubsystem.isReady()) {
            return pinpointSubsystem.getHeadingDegrees();
        }
        return lastPinpointHeadingDegrees;
    }

    /*
     * Back and D-pad Left. Call it once per loop.
     *   Back: if the safety rule switched field-centric off, turn it back ON. Otherwise switch
     *         between FIELD-CENTRIC and ROBOT-CENTRIC.
     *   D-pad Left: RE-AIM. Only works while the Pinpoint is READY (it needs the heading). It also
     *         turns field-centric back on if the safety rule had switched it off.
     */
    private void checkModeButtons() {
        boolean backPressed = gamepad1.back;
        boolean reAimPressed = gamepad1.dpad_left;

        if (isNewPress(backPressed, wasBackPressed)) {
            if (fieldCentricBlocked) {
                // Resume. (If the Pinpoint is still not READY, the safety rule switches it off
                // again on the next loop, and the screen says so.)
                fieldCentricBlocked = false;
                fieldCentricWanted = true;
            } else {
                fieldCentricWanted = !fieldCentricWanted;
            }
        }

        if (isNewPress(reAimPressed, wasReAimPressed) && pinpointSubsystem.isReady()) {
            // Make the heading we drive with ZERO right now: the robot faces away from the driver.
            headingOffsetDegrees = -pinpointSubsystem.getHeadingDegrees();
            fieldCentricBlocked = false;
        }

        wasBackPressed = backPressed;
        wasReAimPressed = reAimPressed;
    }

    /*
     * Is field-centric being used right now? The driver wants it, the safety rule has not switched
     * it off, and the Pinpoint is READY.
     */
    private boolean isFieldCentricActive() {
        return fieldCentricWanted && !fieldCentricBlocked
                && (pinpointSubsystem.isReady() || isPinpointInGlitch());
    }

    /*
     * The words for the "DRIVE MODE" line on the screen.
     */
    private String getDriveModeText() {
        if (!fieldCentricWanted) {
            return "ROBOT-CENTRIC (Back switches to FIELD-CENTRIC)";
        }
        if (fieldCentricBlocked) {
            return "ROBOT-CENTRIC: field-centric was switched off because " + blockedReason
                    + ". Press Back or D-pad Left to turn it on again.";
        }
        if (isPinpointInGlitch()) {
            return String.format("FIELD-CENTRIC (the Pinpoint is not READY: holding the last heading, %.2f of %.2f s)",
                    pinpointNotReadyTimer.seconds(), PINPOINT_GLITCH_TOLERANCE_SECONDS);
        }
        if (!pinpointSubsystem.isReady()) {
            return "ROBOT-CENTRIC for now (the Pinpoint is not READY yet)";
        }
        return "FIELD-CENTRIC (forward = AWAY FROM YOU)";
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The subsystem gets its own motors from the hardware map.
        driveSubsystem = new DriveSubsystem(hardwareMap);

        // The Pinpoint zeros its heading and calibrates its gyro here, so the robot must be STILL
        // during INIT, facing AWAY from the driver. If the configuration has no "pinpoint", this
        // does not stop the OpMode (it just drives robot-centric).
        pinpointSubsystem = new PinpointSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Read the Pinpoint now, so you can see it say READY before START. Back and D-pad Left
        // already work, so you can pick the mode, or re-aim, before you start.
        pinpointSubsystem.update();
        checkModeButtons();

        // The Driver Station clears its screen every loop, so we add the
        // button map again here. That keeps it on the screen until START.
        telemetry.addData("Status", "Ready to start!");
        telemetry.addData("DRIVE MODE", getDriveModeText());
        pinpointSubsystem.addTelemetry(telemetry);
        addRawGamepadTelemetry();
        addControlsTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();

        // The safety rule only counts from START. (While you waited in INIT the Pinpoint may have
        // been calibrating, and that is fine.)
        fieldCentricBlocked = false;
        blockedReason = "";
        haveLastPinpointHeading = false;
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors. (The drive has none. The Pinpoint gives the robot's heading.)
        pinpointSubsystem.update();
        checkPinpointHealth();

        // 2. Read the gamepad and decide what to do.
        checkModeButtons();

        // On the controller, pushing a stick FORWARD gives a negative number, and pushing it RIGHT
        // gives a positive number. Our subsystem wants forward +, LEFT +, and COUNTERCLOCKWISE +,
        // so we flip the sign of all three. (Turning right is clockwise, which is negative.)
        double forward = applyDeadband(-gamepad1.left_stick_y);
        double strafe = applyDeadband(-gamepad1.left_stick_x);
        double turn = applyDeadband(-gamepad1.right_stick_x);

        boolean slowMode = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean testingWheels = gamepad1.x || gamepad1.a || gamepad1.y || gamepad1.b;

        // The heading we drive field-centric with: how far the robot is turned from "facing away
        // from the driver". It is only used when field-centric is active.
        boolean fieldCentricActive = isFieldCentricActive();
        double drivingHeadingDegrees = 0.0;
        if (pinpointSubsystem.isReady() || isPinpointInGlitch()) {
            // (During a short glitch this is the last good heading, not the live one.)
            drivingHeadingDegrees = getPinpointHeadingForDrive() + headingOffsetDegrees;
        }

        // 3. Tell the drivetrain what to do. (There is no update() to call.)
        String driveControl;
        if (testingWheels) {
            // Spin only the wheels whose buttons are held. The other wheels get 0.
            double leftFrontPower = 0.0;
            double leftRearPower = 0.0;
            double rightFrontPower = 0.0;
            double rightRearPower = 0.0;
            if (gamepad1.x) {
                leftFrontPower = WHEEL_TEST_POWER;
            }
            if (gamepad1.a) {
                leftRearPower = WHEEL_TEST_POWER;
            }
            if (gamepad1.y) {
                rightFrontPower = WHEEL_TEST_POWER;
            }
            if (gamepad1.b) {
                rightRearPower = WHEEL_TEST_POWER;
            }
            driveSubsystem.setWheelPowers(leftFrontPower, rightFrontPower,
                    leftRearPower, rightRearPower);
            driveControl = "WHEEL TEST";
        } else {
            // How slow "slow mode" is lives in DriveSubsystem (SLOW_MODE_POWER_SCALE).
            driveSubsystem.setSlowMode(slowMode);
            if (fieldCentricActive) {
                driveSubsystem.driveFieldCentric(forward, strafe, turn, drivingHeadingDegrees);
                driveControl = "Sticks, FIELD-CENTRIC";
            } else {
                driveSubsystem.drive(forward, strafe, turn);
                driveControl = "Sticks, ROBOT-CENTRIC";
            }
            if (slowMode) {
                driveControl = driveControl + " (SLOW MODE)";
            }
        }

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("DRIVE MODE", getDriveModeText());
        telemetry.addData("Control", driveControl);
        telemetry.addData("Heading used for field-centric (deg)", "%.1f   (Pinpoint %.1f + offset %.1f)",
                drivingHeadingDegrees, getPinpointHeadingForDrive(), headingOffsetDegrees);
        telemetry.addData("Pinpoint glitches (came back by themselves)", "%d   longest %.2f s",
                pinpointGlitchCount, pinpointLongestGlitchSeconds);
        pinpointSubsystem.addTelemetry(telemetry);
        addRawGamepadTelemetry();
        driveSubsystem.addTelemetry(telemetry);

        // 5. Keep the button map on the screen while running, at the bottom under the live data.
        //    (Before START the same map is shown by init() and init_loop().)
        if (SHOW_CONTROLS_WHILE_RUNNING) {
            addControlsTelemetry();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        driveSubsystem.stop();
    }
}
