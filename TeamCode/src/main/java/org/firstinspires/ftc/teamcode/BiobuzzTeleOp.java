package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/*
 * TeleOp Alpha 5
 * An iterative OpMode with TWO controllers:
 *   gamepad1 = the DRIVER: drives the robot, runs the INTAKE, and flips the shooter GATE.
 *   gamepad2 = the OPERATOR: runs the shooter FLYWHEEL and HOOD WHEEL, and aims the TURRET.
 * It is DriveAlpha, IntakeAlpha, ShooterRatioAlpha and TurretAlpha put together, so each part
 * has already been tested on its own. This OpMode owns NO hardware. It only reads the gamepads
 * and tells the subsystems what to do.
 *
 * GAMEPAD 2 HAS TWO LAYERS, because the shooter and the turret both want its buttons:
 *   NORMAL (Left Trigger NOT held): the shooter buttons below.
 *   TURRET LAYER (HOLD Left Trigger): Y, X, B, A and the D-pad Left / Right run the turret
 *   instead, and the shooter ignores them. Let go of the trigger to go back to the shooter.
 *   These turret controls work in BOTH layers, so you can aim while you tune the shooter:
 *     Left Stick X     -> turn the turret (push RIGHT and the turret turns RIGHT)
 *     D-pad Up / Down  -> turret POWER +/- 5% (starts at 50%)
 *     Bumpers          -> still reverse the shooter wheels, in either layer
 *   These work ONLY while Left Trigger is held (the TurretAlpha buttons):
 *     Y -> turret CENTER (0 degrees)   X -> FAR LEFT (+90)   B -> FAR RIGHT (-90)
 *     A -> STOP the turret (it floats, so you can turn it by hand)
 *     D-pad Left / Right -> HOLD to turn the turret left / right
 *   Letting go of the stick or the D-pad holds the turret where it is. A preset goes to its
 *   angle and holds. The turret angle limits are OFF for now (see TurretSubsystem), so keep
 *   a hand near the Stop button. 0 degrees is where the turret pointed when the Control Hub
 *   powered on, because the home switch is not installed yet.
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
 *   Right Trigger -> HOLD to run the SHOOTER at a fixed shot: flywheel 45% and hood wheel 90%
 *      power (a ratio of 2.00). While it is held it beats the operator's dials. Let go and the
 *      shooter goes back to whatever gamepad2 set (ON at the dials, or OFF). Reverse (gamepad2
 *      bumpers) still beats it, so clearing a jam always works.
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
 *   Motor name:  turret_motor    (TurretSubsystem, with its encoder cable in the same port)
 */
@TeleOp(name = "TeleOp - Alpha 5", group = "Testing")
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

    // The fixed shot on the driver's Right Trigger, in percent power: flywheel 45% and hood
    // wheel 90%, which is a ratio of 2.00 (hood / flywheel). Change these two numbers to try a
    // different fixed shot. A wheel cannot go past 100%.
    private static final int TRIGGER_SHOT_FLYWHEEL_POWER_PERCENT = 45;
    private static final int TRIGGER_SHOT_HOOD_WHEEL_POWER_PERCENT = 90;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double FLYWHEEL_POWER_REVERSE = -0.60;
    private static final double HOOD_WHEEL_POWER_REVERSE = -0.60;

    // ------------------------------------------------------------------
    // Turret (gamepad2). These are test-only numbers, the same as in TurretAlpha.
    // ------------------------------------------------------------------
    // The turret power at the start, in percent. A full push on the stick, or holding D-pad
    // Left / Right, turns the turret at this power. D-pad Up / Down change it.
    private static final int TURRET_POWER_PERCENT_START = 50;
    private static final int TURRET_POWER_PERCENT_STEP = 5;
    private static final int TURRET_POWER_PERCENT_MIN = 0;
    private static final int TURRET_POWER_PERCENT_MAX = 100;

    // The turret preset angles. LEFT is POSITIVE and RIGHT is NEGATIVE, the same as Pinpoint
    // and Pedro Pathing. Center is 0 degrees, which is 0 encoder ticks.
    private static final double TURRET_PRESET_SIDE_DEGREES = 90.0;
    private static final double TURRET_PRESET_LEFT_DEGREES = TURRET_PRESET_SIDE_DEGREES;
    private static final double TURRET_PRESET_RIGHT_DEGREES = -TURRET_PRESET_SIDE_DEGREES;
    private static final double TURRET_PRESET_CENTER_DEGREES = 0.0;

    // The turret stick must move more than this before it counts.
    private static final double TURRET_STICK_DEADBAND = 0.1;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private TurretSubsystem turretSubsystem;

    // The turret power right now, in percent. D-pad Up and Down (gamepad2) change it.
    private int turretPowerPercent = TURRET_POWER_PERCENT_START;

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
    private boolean wasTriggerShotPressed = false;
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;

    // Remembers whether the turret stick or D-pad was turning the turret last loop, so we can
    // hold the angle when it is released.
    private boolean wasTurningTurretManually = false;

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
        telemetry.addLine(String.format("Right Trigger (hold) = SHOOT at %d%% / %d%% (flywheel / hood)",
                TRIGGER_SHOT_FLYWHEEL_POWER_PERCENT, TRIGGER_SHOT_HOOD_WHEEL_POWER_PERCENT));
        telemetry.addLine("A = TOGGLE shooter gate (loading / shooting)");
        telemetry.addLine("--- GAMEPAD 2 (OPERATOR) ---");
        telemetry.addLine("Y = shooter ON   X = shooter OFF");
        telemetry.addLine("B / A = flywheel power up / down");
        telemetry.addLine("D-pad Right / Left = ratio up / down");
        telemetry.addLine("Right / Left Bumper = reverse flywheel / hood wheel");
        telemetry.addLine("--- GAMEPAD 2 TURRET (works in any layer) ---");
        telemetry.addLine("Left Stick X = turn turret   D-pad Up / Down = turret power +/-");
        telemetry.addLine("--- GAMEPAD 2 TURRET LAYER (HOLD Left Trigger) ---");
        telemetry.addLine(String.format("Y = center (%.0f)  X = far left (%+.0f)  B = far right (%+.0f) degrees",
                TURRET_PRESET_CENTER_DEGREES, TURRET_PRESET_LEFT_DEGREES, TURRET_PRESET_RIGHT_DEGREES));
        telemetry.addLine("A = STOP turret   D-pad Left / Right = hold to turn");
        telemetry.addLine("(The shooter ignores Y, X, B, A and D-pad Left / Right while the trigger is held)");
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
        // The turret does not move at INIT. It only starts reading its encoder.
        turretSubsystem = new TurretSubsystem(hardwareMap);

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

        // The turret is not driven during INIT. We only read it, so you can turn it by hand
        // and watch the angle.
        turretSubsystem.update();
        turretSubsystem.addTelemetry(telemetry);
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
        boolean triggerShotPressed = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;

        // The wheels change speed when the trigger goes down AND when it comes back up,
        // so both start the "wheels settled?" count over.
        if (triggerShotPressed != wasTriggerShotPressed) {
            restartSpinTimer();
        }

        // ---- Gamepad 2 (operator) ----
        boolean yPressed = gamepad2.y;
        boolean xPressed = gamepad2.x;
        boolean bPressed = gamepad2.b;
        boolean aPressed = gamepad2.a;
        boolean dpadLeftPressed = gamepad2.dpad_left;
        boolean dpadRightPressed = gamepad2.dpad_right;

        // Holding the Left Trigger switches gamepad2 to the TURRET LAYER. In that layer these
        // buttons belong to the turret, so the shooter must ignore them. (We still remember
        // them at the bottom of loop(), so letting go of the trigger while a button is still
        // held does not count as a new press.)
        boolean turretLayer = gamepad2.left_trigger > TRIGGER_PRESSED_THRESHOLD;

        // Shooter on and off. If both are pressed at once, OFF wins (the safer choice).
        if (!turretLayer && isNewPress(yPressed, wasYPressed)) {
            shooterOn = true;
            restartSpinTimer();
        }
        if (!turretLayer && isNewPress(xPressed, wasXPressed)) {
            shooterOn = false;
        }

        // Turn the dials. Any change makes the wheels change speed, so start the count over.
        if (!turretLayer && isNewPress(bPressed, wasBPressed)) {
            flywheelPowerPercent = flywheelPowerPercent + FLYWHEEL_POWER_PERCENT_STEP;
            restartSpinTimer();
        }
        if (!turretLayer && isNewPress(aPressed, wasAPressed)) {
            flywheelPowerPercent = flywheelPowerPercent - FLYWHEEL_POWER_PERCENT_STEP;
            restartSpinTimer();
        }
        if (!turretLayer && isNewPress(dpadRightPressed, wasDpadRightPressed)) {
            ratioHundredths = ratioHundredths + RATIO_HUNDREDTHS_STEP;
            restartSpinTimer();
        }
        if (!turretLayer && isNewPress(dpadLeftPressed, wasDpadLeftPressed)) {
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
        } else if (triggerShotPressed) {
            // The driver's fixed shot beats the operator's dials while the trigger is held.
            shooterSubsystem.setPower(
                    TRIGGER_SHOT_FLYWHEEL_POWER_PERCENT / PERCENT_PER_ONE,
                    TRIGGER_SHOT_HOOD_WHEEL_POWER_PERCENT / PERCENT_PER_ONE);
            shooterControl = String.format("Right Trigger (%d / %d)",
                    TRIGGER_SHOT_FLYWHEEL_POWER_PERCENT, TRIGGER_SHOT_HOOD_WHEEL_POWER_PERCENT);
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

        // Turret (operator, gamepad2). The same controls as TurretAlpha. The first one that is
        // true wins. The stick and the power dial work in either layer. The presets, STOP, and
        // the D-pad Left / Right turning work only in the TURRET LAYER (Left Trigger held).

        // D-pad Up and Down change the turret power. Only a NEW press counts.
        boolean dpadUpPressed = gamepad2.dpad_up;
        boolean dpadDownPressed = gamepad2.dpad_down;
        if (isNewPress(dpadUpPressed, wasDpadUpPressed)) {
            turretPowerPercent = turretPowerPercent + TURRET_POWER_PERCENT_STEP;
        }
        if (isNewPress(dpadDownPressed, wasDpadDownPressed)) {
            turretPowerPercent = turretPowerPercent - TURRET_POWER_PERCENT_STEP;
        }
        if (turretPowerPercent > TURRET_POWER_PERCENT_MAX) {
            turretPowerPercent = TURRET_POWER_PERCENT_MAX;
        }
        if (turretPowerPercent < TURRET_POWER_PERCENT_MIN) {
            turretPowerPercent = TURRET_POWER_PERCENT_MIN;
        }

        // Turn the percent into a number from 0.0 to 1.0.
        double turretPower = turretPowerPercent / PERCENT_PER_ONE;

        // Stick RIGHT is a positive number on the controller. But turning RIGHT means going
        // clockwise, and clockwise is a NEGATIVE angle. So we flip the sign of the stick.
        double turretStick = -gamepad2.left_stick_x;
        boolean usingTurretStick = Math.abs(turretStick) > TURRET_STICK_DEADBAND;

        // The D-pad turns the turret only while it is held, and only in the turret layer.
        // Pressing Left and Right together cancels out, so neither counts.
        boolean turretDpadLeftOnly = turretLayer && dpadLeftPressed && !dpadRightPressed;
        boolean turretDpadRightOnly = turretLayer && dpadRightPressed && !dpadLeftPressed;
        boolean turningTurretManually =
                usingTurretStick || turretDpadLeftOnly || turretDpadRightOnly;

        String turretControl;
        if (usingTurretStick) {
            // The stick scales the power: a full push is the whole turret power,
            // a half push is half of it.
            turretSubsystem.setManualPower(turretStick * turretPower);
            turretControl = "Left Stick (MANUAL)";
        } else if (turretDpadLeftOnly) {
            // Left is a POSITIVE angle, so positive power.
            turretSubsystem.setManualPower(turretPower);
            turretControl = "D-pad Left (MANUAL)";
        } else if (turretDpadRightOnly) {
            turretSubsystem.setManualPower(-turretPower);
            turretControl = "D-pad Right (MANUAL)";
        } else if (wasTurningTurretManually) {
            // The stick or D-pad was just let go, so hold the turret where it stopped.
            turretSubsystem.holdCurrentAngle();
            turretControl = "Released (HOLD)";
        } else if (turretLayer && yPressed) {
            turretSubsystem.setTargetAngle(TURRET_PRESET_CENTER_DEGREES);
            turretControl = "Y (preset CENTER)";
        } else if (turretLayer && xPressed) {
            turretSubsystem.setTargetAngle(TURRET_PRESET_LEFT_DEGREES);
            turretControl = "X (preset FAR LEFT)";
        } else if (turretLayer && bPressed) {
            turretSubsystem.setTargetAngle(TURRET_PRESET_RIGHT_DEGREES);
            turretControl = "B (preset FAR RIGHT)";
        } else if (turretLayer && aPressed) {
            turretSubsystem.stop();
            turretControl = "A (STOP)";
        } else {
            // No new command. The turret keeps doing what it was last told to do.
            turretControl = "None (keeps last command)";
        }
        turretSubsystem.update();

        // Remember this loop's buttons for the next loop.
        wasDpadUpPressed = dpadUpPressed;
        wasDpadDownPressed = dpadDownPressed;
        wasTurningTurretManually = turningTurretManually;
        wasGateButtonPressed = gateButtonPressed;
        wasYPressed = yPressed;
        wasXPressed = xPressed;
        wasBPressed = bPressed;
        wasAPressed = aPressed;
        wasDpadLeftPressed = dpadLeftPressed;
        wasDpadRightPressed = dpadRightPressed;
        wasTriggerShotPressed = triggerShotPressed;

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
        if (!shooterOn && !triggerShotPressed) {
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

        // The turret prints its own "--- TURRET ---" heading, so we add our lines under it.
        turretSubsystem.addTelemetry(telemetry);
        telemetry.addData("Turret Control", turretControl);
        telemetry.addData("Turret power", "%d%%", turretPowerPercent);
        telemetry.addData("Gamepad2 layer", turretLayer ? "TURRET (Left Trigger held)" : "SHOOTER");
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
        turretSubsystem.stop();
    }
}
