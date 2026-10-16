package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/*
 * TeleOp Alpha 4
 * An iterative OpMode with TWO controllers:
 *   gamepad1 = the DRIVER: drives the robot, runs the INTAKE, and flips the shooter GATE.
 *   gamepad2 = the OPERATOR: runs the shooter FLYWHEEL and HOOD WHEEL.
 * It is DriveAlpha, IntakeAlpha and ShooterRatioAlpha put together, so each part has already
 * been tested on its own. This OpMode owns NO hardware. It only reads the gamepads and tells
 * the subsystems what to do. (The turret will be added in a later alpha.)
 *
 * It starts ROBOT-CENTRIC: "forward" means the way the robot is facing.
 *
 * GAMEPAD 1 - DRIVER
 *   Left Stick   -> drive forward / backward and slide (strafe) left / right
 *   Right Stick  -> turn (push right to turn right)
 *   HOLD the Left Trigger -> SLOW MODE for careful lining up
 *   Right Bumper -> INTAKE (pull balls in)      Left Bumper -> REVERSE (push balls out)
 *      (Left Bumper beats Right Bumper, so clearing a jam always works. The intake runs at the
 *       power set in IntakeSubsystem, DEFAULT_INTAKE_POWER.)
 *   A (Cross)    -> TOGGLES the shooter gate between LOADING and SHOOTING (one flip per press)
 *      (The gate positions are LOADING_POSITION and SHOOTING_POSITION in ShooterServoSubsystem.)
 *
 * GAMEPAD 2 - OPERATOR (shooter, plain POWER because the encoders are not connected yet)
 *   Y            -> shooter ON (spins at the dialed powers and keeps spinning)
 *   X            -> shooter OFF (the wheels coast down)
 *   B / A        -> flywheel power UP / DOWN by FLYWHEEL_POWER_PERCENT_STEP
 *   D-pad Right / Left -> ratio UP / DOWN by 0.05
 *   Right Bumper -> HOLD to reverse the flywheel only (clearing a jam)
 *   Left Bumper  -> HOLD to reverse the hood wheel only (clearing a jam)
 *   The hood wheel power is flywheel power x ratio (RATIO = HOOD WHEEL / FLYWHEEL). A wheel
 *   cannot go past 100%, so if the hood wheel would be over 100% the screen says CAPPED.
 *   The screen counts 3 seconds for "Wheels settled?" because we cannot read wheel speed yet.
 *   It starts over after the shooter starts, a dial change, a reverse, and every gate flip.
 *   ShooterRatioAlpha has the same shooter controls on their own, for testing.
 *
 * THE INTAKE ALSO RUNS THE TRANSFER. One motor spins the front intake AND a hex shaft with
 * grippers in the transfer chute, so balls are carried up the chute while you drive.
 *
 * NOT IN THIS OPMODE: the one-wheel direction test. Use DriveAlpha for that, with the
 * robot on blocks.
 *
 * SAFE USE: keep hands, hair and loose clothing away from the shooter wheels, and keep
 * everyone out of the path of the ball.
 *
 * WHY THE STICK SIGNS ARE FLIPPED
 *   The drive subsystem uses the same convention as Pedro Pathing, Pinpoint and the turret:
 *   forward +, strafe LEFT +, turn COUNTERCLOCKWISE +. The controller's sticks are different:
 *   pushing a stick forward gives a NEGATIVE number, and pushing it right gives a POSITIVE
 *   number. So we flip the sign of all three.
 *
 * On the Driver Station, pair the controllers with Start + A (gamepad1) and Start + B (gamepad2).
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle, bumpers = L1 / R1,
 * triggers = L2 / R2.
 *
 * Robot configuration (on the Driver Station). The names live in the subsystem classes:
 *   Motor names: left_front, left_rear, right_front, right_rear   (DriveSubsystem)
 *   Motor name:  intake                                           (IntakeSubsystem)
 *   Motor names: flywheel, hood_wheel                             (ShooterSubsystem)
 *   Servo name:  shooter_servo   (ShooterServoSubsystem, goBILDA dual mode, POSITIONAL mode)
 */
@TeleOp(name = "TeleOp - Alpha 4", group = "Testing")
public class BiobuzzTeleOp extends OpMode {

    // ------------------------------------------------------------------
    // Driving
    // ------------------------------------------------------------------
    // The sticks do not always rest at exactly zero. Anything smaller than this is ignored,
    // so the robot does not creep when you let go.
    private static final double STICK_DEADBAND = 0.05;

    // The trigger gives a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // ------------------------------------------------------------------
    // Shooter dials (gamepad2). These are test-only numbers.
    // ------------------------------------------------------------------
    // The flywheel power dial is counted in WHOLE PERCENT (30 means 30% power).
    // Where it starts, how far one press moves it, and how low and high it can go.
    private static final int FLYWHEEL_POWER_PERCENT_START = 30;
    private static final int FLYWHEEL_POWER_PERCENT_STEP = 5;
    private static final int FLYWHEEL_POWER_PERCENT_MIN = 5;
    private static final int FLYWHEEL_POWER_PERCENT_MAX = 100;

    // The ratio dial is counted in HUNDREDTHS so that adding 5 over and over never drifts to
    // something like 1.9999999. So 200 means a ratio of 2.00, and a step of 5 means 0.05.
    private static final int RATIO_HUNDREDTHS_START = 200;
    private static final int RATIO_HUNDREDTHS_STEP = 5;
    private static final int RATIO_HUNDREDTHS_MIN = 100;
    private static final int RATIO_HUNDREDTHS_MAX = 300;

    // How many hundredths make a whole number (200 hundredths = 2.00).
    private static final double HUNDREDTHS_PER_ONE = 100.0;

    // How many percent make a whole power of 1.0 (100 percent = 1.0 power).
    private static final double PERCENT_PER_ONE = 100.0;

    // The most power a wheel can be given, in percent. A hood wheel power above this is CAPPED.
    private static final double MAX_POWER_PERCENT = 100.0;

    // How long the wheels need to settle after the shooter starts, a dial changes, or a ball
    // goes through. A starting guess. Time it with a stopwatch.
    private static final double SPIN_SETTLE_SECONDS = 3.0;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double FLYWHEEL_POWER_REVERSE = -0.60;
    private static final double HOOD_WHEEL_POWER_REVERSE = -0.60;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;

    // The shooter dials, in whole numbers.
    private int flywheelPowerPercent = FLYWHEEL_POWER_PERCENT_START;
    private int ratioHundredths = RATIO_HUNDREDTHS_START;

    // True after Y, false after X. The shooter keeps spinning between button presses.
    private boolean shooterOn = false;

    // Times how long since the wheels last had a reason to change speed.
    private ElapsedTime spinTimer = new ElapsedTime();

    // What the buttons were doing LAST loop. A press lasts many loops (about 50 per second),
    // but we want it to count only ONCE. So we only act when it was NOT pressed last loop
    // and IS pressed now.
    private boolean wasGateButtonPressed = false;
    private boolean wasYPressed = false;
    private boolean wasXPressed = false;
    private boolean wasBPressed = false;
    private boolean wasAPressed = false;
    private boolean wasDpadLeftPressed = false;
    private boolean wasDpadRightPressed = false;

    /*
     * True only on the loop where a button goes from "not pressed" to "pressed".
     */
    private boolean isNewPress(boolean pressedNow, boolean pressedLastLoop) {
        return pressedNow && !pressedLastLoop;
    }

    /*
     * Start the "wheels settled?" count over.
     */
    private void restartSpinTimer() {
        spinTimer.reset();
    }

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- GAMEPAD 1 (DRIVER) ---");
        telemetry.addLine("Left Stick = drive and strafe   Right Stick = turn");
        telemetry.addLine("Left Trigger = SLOW MODE");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
        telemetry.addLine("A = TOGGLE shooter gate (loading / shooting)");
        telemetry.addLine("--- GAMEPAD 2 (OPERATOR) ---");
        telemetry.addLine("Y = shooter ON   X = shooter OFF");
        telemetry.addLine("B / A = flywheel power up / down");
        telemetry.addLine("D-pad Right / Left = ratio up / down");
        telemetry.addLine("Right / Left Bumper = reverse flywheel / hood wheel");
    }

    /*
     * Shows the RAW numbers from gamepad1, before any deadband or sign flip.
     * This is for finding a controller problem: with your hands OFF the controller, every stick
     * should read about 0.00, both triggers 0.00, and every button false. Anything else means
     * the controller (or how the Driver Station sees it) is the problem, not the drive code.
     * It is only shown before START, so it does not crowd the screen while driving.
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
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Each subsystem gets its own motors from the hardware map.
        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        // The gate servo moves to LOADING here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // The Driver Station clears its screen every loop, so we add the
        // button map again here. That keeps it on the screen until START.
        telemetry.addData("Status", "Ready to start!");
        addRawGamepadTelemetry();
        addControlsTelemetry();

        // Show the gate here too, so you can check before START that it went to LOADING.
        // (The position shown is the one we COMMANDED. A servo cannot report where it really is.)
        shooterServoSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        restartSpinTimer();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors. (The drive has none. The intake reads its own encoder in update().
        //    The shooter has no working encoders yet.)
        intakeSubsystem.update();

        // 2. Read the gamepads and decide what to do.

        // ---- Gamepad 1 (driver) ----
        // On the controller, pushing a stick FORWARD gives a negative number, and pushing it RIGHT
        // gives a positive number. Our subsystem wants forward +, LEFT +, and COUNTERCLOCKWISE +,
        // so we flip the sign of all three. (Turning right is clockwise, which is negative.)
        double forward = applyDeadband(-gamepad1.left_stick_y);
        double strafe = applyDeadband(-gamepad1.left_stick_x);
        double turn = applyDeadband(-gamepad1.right_stick_x);

        boolean slowMode = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean gateButtonPressed = gamepad1.a;

        // ---- Gamepad 2 (operator) ----
        boolean yPressed = gamepad2.y;
        boolean xPressed = gamepad2.x;
        boolean bPressed = gamepad2.b;
        boolean aPressed = gamepad2.a;
        boolean dpadLeftPressed = gamepad2.dpad_left;
        boolean dpadRightPressed = gamepad2.dpad_right;

        // Shooter on and off. If both are pressed at once, OFF wins (the safer choice).
        if (isNewPress(yPressed, wasYPressed)) {
            shooterOn = true;
            restartSpinTimer();
        }
        if (isNewPress(xPressed, wasXPressed)) {
            shooterOn = false;
        }

        // Turn the dials. Any change makes the wheels change speed, so start the count over.
        if (isNewPress(bPressed, wasBPressed)) {
            flywheelPowerPercent = flywheelPowerPercent + FLYWHEEL_POWER_PERCENT_STEP;
            restartSpinTimer();
        }
        if (isNewPress(aPressed, wasAPressed)) {
            flywheelPowerPercent = flywheelPowerPercent - FLYWHEEL_POWER_PERCENT_STEP;
            restartSpinTimer();
        }
        if (isNewPress(dpadRightPressed, wasDpadRightPressed)) {
            ratioHundredths = ratioHundredths + RATIO_HUNDREDTHS_STEP;
            restartSpinTimer();
        }
        if (isNewPress(dpadLeftPressed, wasDpadLeftPressed)) {
            ratioHundredths = ratioHundredths - RATIO_HUNDREDTHS_STEP;
            restartSpinTimer();
        }

        // Keep the dials between their lowest and highest settings.
        if (flywheelPowerPercent > FLYWHEEL_POWER_PERCENT_MAX) {
            flywheelPowerPercent = FLYWHEEL_POWER_PERCENT_MAX;
        }
        if (flywheelPowerPercent < FLYWHEEL_POWER_PERCENT_MIN) {
            flywheelPowerPercent = FLYWHEEL_POWER_PERCENT_MIN;
        }
        if (ratioHundredths > RATIO_HUNDREDTHS_MAX) {
            ratioHundredths = RATIO_HUNDREDTHS_MAX;
        }
        if (ratioHundredths < RATIO_HUNDREDTHS_MIN) {
            ratioHundredths = RATIO_HUNDREDTHS_MIN;
        }

        // The hood wheel power is the flywheel power times the ratio. A wheel cannot go past
        // 100%, so if the answer is more than that we use 100% and warn that it was CAPPED.
        double ratio = ratioHundredths / HUNDREDTHS_PER_ONE;
        double wantedHoodWheelPercent = flywheelPowerPercent * ratio;
        boolean hoodWheelCapped = wantedHoodWheelPercent > MAX_POWER_PERCENT;
        double hoodWheelPercent = Math.min(wantedHoodWheelPercent, MAX_POWER_PERCENT);

        // Turn the percents into the 0.0 to 1.0 powers the subsystem wants (30 -> 0.30).
        double flywheelPower = flywheelPowerPercent / PERCENT_PER_ONE;
        double hoodWheelPower = hoodWheelPercent / PERCENT_PER_ONE;

        // 3. Tell the subsystems what to do. (The drive has no update() to call.)

        // Drive.
        // How slow "slow mode" is lives in DriveSubsystem (SLOW_MODE_POWER_SCALE).
        driveSubsystem.setSlowMode(slowMode);
        String driveControl;
        if (slowMode) {
            driveControl = "SLOW MODE";
        } else {
            driveControl = "Normal";
        }
        driveSubsystem.drive(forward, strafe, turn);

        // Intake (driver bumpers). Check REVERSE first, so clearing a jam always works.
        String intakeControl;
        if (gamepad1.left_bumper) {
            intakeSubsystem.reverse();
            intakeControl = "Left Bumper (REVERSE)";
        } else if (gamepad1.right_bumper) {
            intakeSubsystem.intake();
            intakeControl = "Right Bumper (INTAKE)";
        } else {
            // No bumper held, so stop the intake.
            intakeControl = "None";
            intakeSubsystem.stop();
        }

        // Shooter wheels (operator). Reverse beats everything, so clearing a jam always works.
        String shooterControl;
        boolean reverseFlywheel = gamepad2.right_bumper;
        boolean reverseHoodWheel = gamepad2.left_bumper;
        if (reverseFlywheel || reverseHoodWheel) {
            // A wheel that is NOT being reversed gets 0 power.
            double reverseFlywheelPower = 0.0;
            double reverseHoodWheelPower = 0.0;
            if (reverseFlywheel) {
                reverseFlywheelPower = FLYWHEEL_POWER_REVERSE;
            }
            if (reverseHoodWheel) {
                reverseHoodWheelPower = HOOD_WHEEL_POWER_REVERSE;
            }
            shooterSubsystem.setPower(reverseFlywheelPower, reverseHoodWheelPower);
            shooterControl = "Bumper (REVERSE)";
            // The wheels need time to settle again after a reverse.
            restartSpinTimer();
        } else if (shooterOn) {
            shooterSubsystem.setPower(flywheelPower, hoodWheelPower);
            shooterControl = "ON";
        } else {
            shooterSubsystem.stop();
            shooterControl = "OFF";
        }
        shooterSubsystem.update();

        // Shooter gate (driver). A counts as ONE press only on the loop where it goes from
        // "not pressed" to "pressed". Without this check, holding A would flip the gate every
        // loop. Flipping it lets a ball through, which slows the wheels, so every flip also
        // starts the "wheels settled?" count over.
        if (isNewPress(gateButtonPressed, wasGateButtonPressed)) {
            shooterServoSubsystem.toggle();
            restartSpinTimer();
        }

        // Remember this loop's buttons for the next loop.
        wasGateButtonPressed = gateButtonPressed;
        wasYPressed = yPressed;
        wasXPressed = xPressed;
        wasBPressed = bPressed;
        wasAPressed = aPressed;
        wasDpadLeftPressed = dpadLeftPressed;
        wasDpadRightPressed = dpadRightPressed;

        // 4. Show what is happening on the Driver Station, one section at a time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive Control", driveControl);
        telemetry.addData("Drive inputs", "forward %.2f   strafe %.2f   turn %.2f",
                forward, strafe, turn);

        telemetry.addLine("--- SHOOTER (power) ---");
        telemetry.addData("Flywheel power", "%d%%", flywheelPowerPercent);
        telemetry.addData("Ratio (hood / flywheel)", "%.2f", ratio);
        if (hoodWheelCapped) {
            telemetry.addData("Hood wheel power", "%.0f%%  CAPPED (you asked for %.0f%%)",
                    hoodWheelPercent, wantedHoodWheelPercent);
        } else {
            telemetry.addData("Hood wheel power", "%.0f%%", hoodWheelPercent);
        }
        telemetry.addData("Shooter switch", shooterControl);
        String settledText;
        if (!shooterOn) {
            settledText = "shooter is OFF";
        } else if (spinTimer.seconds() >= SPIN_SETTLE_SECONDS) {
            settledText = "YES - OK to shoot";
        } else {
            settledText = String.format("WAIT (%.1f of %.0f seconds)",
                    spinTimer.seconds(), SPIN_SETTLE_SECONDS);
        }
        telemetry.addData("Wheels settled?", settledText);

        // The gate prints its own "--- SHOOTER GATE ---" heading.
        shooterServoSubsystem.addTelemetry(telemetry);

        // The intake prints its own "--- INTAKE ---" heading, so we add our line under it.
        intakeSubsystem.addTelemetry(telemetry);
        telemetry.addData("Intake Control", intakeControl);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        // The gate servo just stays where it is. A servo holds its position by itself.
        driveSubsystem.stop();
        intakeSubsystem.stop();
        shooterSubsystem.stop();
    }
}
