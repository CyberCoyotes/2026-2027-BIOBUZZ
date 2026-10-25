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
 *   gamepad1 = the DRIVER: drives the robot, runs the INTAKE, and holds the trigger to SHOOT.
 *   gamepad2 = the OPERATOR: sets the shooter speeds (RPM), aims the TURRET, and moves the GATE.
 * It is DriveAlpha, IntakeAlpha, ShooterRPMAlpha and TurretAlpha put together, so each part
 * has already been tested on its own. This OpMode owns NO hardware. It only reads the gamepads
 * and tells the subsystems what to do.
 *
 * GAMEPAD 2 (the operator) has NO layers. Every button always does the same thing:
 *   Left Stick X     -> turn the turret (push RIGHT and the turret turns RIGHT)
 *   Y -> turret CENTER (0 degrees)   X -> FAR LEFT (+90)   B -> FAR RIGHT (-90)
 *   A -> STOP the turret (it floats, so you can turn it by hand)
 *   D-pad Up / Down  -> flywheel target RPM +/- FLYWHEEL_RPM_STEP
 *   D-pad Right / Left -> ratio (hood wheel / flywheel) +/- 0.05
 *   Left Bumper      -> HOLD to reverse BOTH shooter wheels (clearing a jam)
 *   Right Bumper     -> HOLD to reverse the flywheel only
 *   Right Trigger    -> shooter GATE to SHOOTING (lets the ball through)
 *   Left Trigger     -> shooter GATE to LOADING (holds the ball back)
 *      (One move per press. The gate positions are LOADING_POSITION and SHOOTING_POSITION
 *       in ShooterServoSubsystem. If both triggers are pressed at once, LOADING wins.)
 *   Letting go of the stick holds the turret where it is. A preset goes to its angle and
 *   holds. The turret will not go past +/-120 degrees (see TurretSubsystem). 0 degrees is where
 *   the turret pointed when the Control Hub powered on, because the home switch is not
 *   installed yet, so power on with the turret pointing straight ahead.
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
 *   Right Trigger -> HOLD to run the SHOOTER at the speeds the operator has dialed in (see
 *      GAMEPAD 2 below). At the start that is flywheel 2700 RPM and hood wheel 5400 RPM (a
 *      ratio of 2.00). Let go and both wheels go to 0 power and coast down.
 *      Reverse (gamepad2 bumpers) still beats it, so clearing a jam always works.
 *
 * GAMEPAD 2 - OPERATOR, SHOOTER DIALS (SPEED CONTROL in RPM, read from the wheel encoders)
 *   D-pad Up / Down    -> flywheel target RPM UP / DOWN by FLYWHEEL_RPM_STEP
 *   D-pad Right / Left -> ratio UP / DOWN by 0.05
 *   Left Bumper  -> HOLD to reverse BOTH the flywheel and the hood wheel (clearing a jam)
 *   Right Bumper -> HOLD to reverse the flywheel only
 *   The hood wheel target RPM is flywheel RPM x ratio (RATIO = HOOD WHEEL / FLYWHEEL). The
 *   shooter will not go past its MAX_TARGET_RPM, so if the hood wheel would be over that, the
 *   screen says CAPPED and the real ratio is lower than the dial says.
 *   Jam clearing (the bumpers) still uses plain negative POWER, because speed control only
 *   works forward.
 *   The screen shows "OK to shoot" only when BOTH wheels are really at their target RPM
 *   (the shooter's isAtSpeed()). It drops back to WAIT after a ball slows the wheels.
 *   ShooterRPMAlpha has the same wheel speed control on its own, for testing.
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
    // The flywheel dial is counted in WHOLE RPM (2700 means 2700 RPM).
    // Where it starts, how far one press moves it, and how low and high it can go.
    // It starts at 2700 RPM and the ratio starts at 2.00, so the hood wheel starts at 5400 RPM.
    // 2700 is only an ESTIMATE: it is the old 45% power start x 6000 RPM (the motor's top speed
    // with no load). Tune it on the robot, then type the best numbers in here.
    private static final int FLYWHEEL_RPM_START = 2700;
    private static final int FLYWHEEL_RPM_STEP = 100;
    private static final int FLYWHEEL_RPM_MIN = 500;
    private static final int FLYWHEEL_RPM_MAX = 3600;

    // The ratio dial is counted in HUNDREDTHS so that adding 5 over and over never drifts to
    // something like 1.9999999. So 200 means a ratio of 2.00, and a step of 5 means 0.05.
    private static final int RATIO_HUNDREDTHS_START = 200;
    private static final int RATIO_HUNDREDTHS_STEP = 5;
    private static final int RATIO_HUNDREDTHS_MIN = 100;
    private static final int RATIO_HUNDREDTHS_MAX = 300;

    // How many hundredths make a whole number (200 hundredths = 2.00).
    private static final double HUNDREDTHS_PER_ONE = 100.0;

    // How many percent make a whole power of 1.0 (100 percent = 1.0 power).
    // The turret power below is still counted in percent.
    private static final double PERCENT_PER_ONE = 100.0;

    // A hood wheel target counts as CAPPED when it is higher than the shooter's maximum by more
    // than this many RPM. (A little room, so a tiny rounding error does not cry wolf.)
    private static final double CAPPED_TOLERANCE_RPM = 1.0;

    // The measured ratio (hood RPM / flywheel RPM) is only worth showing when the flywheel is
    // spinning faster than this. Below it the division gives silly numbers.
    private static final double MIN_RPM_FOR_MEASURED_RATIO = 300.0;

    // Reverse power is NEGATIVE so the motor spins the other way. Jam clearing is plain power,
    // not RPM, because speed control in ShooterSubsystem only goes forward.
    private static final double FLYWHEEL_POWER_REVERSE = -0.60;
    private static final double HOOD_WHEEL_POWER_REVERSE = -0.60;

    // ------------------------------------------------------------------
    // Turret (gamepad2). These are test-only numbers, the same as in TurretAlpha.
    // ------------------------------------------------------------------
    // The turret power, in percent. A full push on the stick turns the turret at this power.
    // (D-pad Up / Down used to change it. They now change the flywheel RPM instead.)
    private static final int TURRET_POWER_PERCENT = 100;

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

    // The shooter dials, in whole numbers.
    private int flywheelRpmDial = FLYWHEEL_RPM_START;
    private int ratioHundredths = RATIO_HUNDREDTHS_START;

    // What the buttons were doing LAST loop. A press lasts many loops (about 50 per second),
    // but we want it to count only ONCE. So we only act when it was NOT pressed last loop
    // and IS pressed now.
    private boolean wasGateShootTriggerPressed = false;
    private boolean wasGateLoadTriggerPressed = false;
    private boolean wasDpadLeftPressed = false;
    private boolean wasDpadRightPressed = false;
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;

    // Remembers whether the turret stick was turning the turret last loop, so we can
    // hold the angle when it is released.
    private boolean wasTurningTurretManually = false;

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
        telemetry.addLine("--- GAMEPAD 1 (DRIVER) ---");
        telemetry.addLine("Left Stick = drive and strafe   Right Stick = turn");
        telemetry.addLine("Left Trigger = SLOW MODE");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
        telemetry.addLine("Right Trigger (hold) = SHOOT at the dialed RPMs");
        telemetry.addLine("--- GAMEPAD 2 (OPERATOR) ---");
        telemetry.addLine("D-pad Up / Down = flywheel RPM up / down");
        telemetry.addLine("D-pad Right / Left = ratio up / down");
        telemetry.addLine("Left Bumper = reverse BOTH wheels   Right Bumper = reverse flywheel only");
        telemetry.addLine("Left Stick X = turn turret");
        telemetry.addLine(String.format("Y = center (%.0f)  X = far left (%+.0f)  B = far right (%+.0f) degrees",
                TURRET_PRESET_CENTER_DEGREES, TURRET_PRESET_LEFT_DEGREES, TURRET_PRESET_RIGHT_DEGREES));
        telemetry.addLine("A = STOP turret");
        telemetry.addLine("Right Trigger = gate SHOOTING   Left Trigger = gate LOADING");
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors. (The drive has none. The intake reads its own encoder in update().
        //    The shooter reads its wheel encoders in its own update(), in step 3 below.)
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
        boolean triggerShotPressed = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;

        // ---- Gamepad 2 (operator) ----
        boolean yPressed = gamepad2.y;
        boolean xPressed = gamepad2.x;
        boolean bPressed = gamepad2.b;
        boolean aPressed = gamepad2.a;
        boolean dpadUpPressed = gamepad2.dpad_up;
        boolean dpadDownPressed = gamepad2.dpad_down;
        boolean dpadLeftPressed = gamepad2.dpad_left;
        boolean dpadRightPressed = gamepad2.dpad_right;
        boolean gateShootTriggerPressed = gamepad2.right_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean gateLoadTriggerPressed = gamepad2.left_trigger > TRIGGER_PRESSED_THRESHOLD;

        // Turn the dials. Only a NEW press counts, so holding a button moves it one step.
        if (isNewPress(dpadUpPressed, wasDpadUpPressed)) {
            flywheelRpmDial = flywheelRpmDial + FLYWHEEL_RPM_STEP;
        }
        if (isNewPress(dpadDownPressed, wasDpadDownPressed)) {
            flywheelRpmDial = flywheelRpmDial - FLYWHEEL_RPM_STEP;
        }
        if (isNewPress(dpadRightPressed, wasDpadRightPressed)) {
            ratioHundredths = ratioHundredths + RATIO_HUNDREDTHS_STEP;
        }
        if (isNewPress(dpadLeftPressed, wasDpadLeftPressed)) {
            ratioHundredths = ratioHundredths - RATIO_HUNDREDTHS_STEP;
        }

        // Keep the dials between their lowest and highest settings.
        if (flywheelRpmDial > FLYWHEEL_RPM_MAX) {
            flywheelRpmDial = FLYWHEEL_RPM_MAX;
        }
        if (flywheelRpmDial < FLYWHEEL_RPM_MIN) {
            flywheelRpmDial = FLYWHEEL_RPM_MIN;
        }
        if (ratioHundredths > RATIO_HUNDREDTHS_MAX) {
            ratioHundredths = RATIO_HUNDREDTHS_MAX;
        }
        if (ratioHundredths < RATIO_HUNDREDTHS_MIN) {
            ratioHundredths = RATIO_HUNDREDTHS_MIN;
        }

        // The hood wheel target is the flywheel target times the ratio. The shooter will not go
        // past its maximum RPM, so if the answer is more than that we show the maximum and warn
        // that it was CAPPED. (The shooter cuts it down by itself, we only warn about it.)
        double ratio = ratioHundredths / HUNDREDTHS_PER_ONE;
        double flywheelTargetRpm = flywheelRpmDial;
        double wantedHoodWheelRpm = flywheelTargetRpm * ratio;
        double maxTargetRpm = shooterSubsystem.getMaxTargetRpm();
        boolean hoodWheelCapped = wantedHoodWheelRpm > maxTargetRpm + CAPPED_TOLERANCE_RPM;
        double hoodWheelTargetRpm = Math.min(wantedHoodWheelRpm, maxTargetRpm);

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
        // Left Bumper reverses BOTH wheels. Right Bumper reverses only the flywheel.
        boolean reverseFlywheel = gamepad2.left_bumper || gamepad2.right_bumper;
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
        } else if (triggerShotPressed) {
            // The driver holds the Right Trigger to shoot, at the RPMs the operator dialed in.
            // The shooter's speed control holds each wheel at its target, even when a ball
            // slows it down or the battery drains.
            shooterSubsystem.setTargetRpms(flywheelTargetRpm, hoodWheelTargetRpm);
            shooterControl = "Right Trigger (SHOOT)";
        } else {
            // Trigger not held, so both wheels get 0 power and coast down.
            shooterSubsystem.stop();
            shooterControl = "OFF";
        }
        shooterSubsystem.update();

        // Shooter gate (operator). A trigger counts as ONE press only on the loop where it goes
        // from "not pressed" to "pressed", so holding it does not keep re-sending the move.
        // Left Trigger is checked first, so if both are pressed at once the gate goes to LOADING.
        if (isNewPress(gateLoadTriggerPressed, wasGateLoadTriggerPressed)) {
            shooterServoSubsystem.goToLoading();
        } else if (isNewPress(gateShootTriggerPressed, wasGateShootTriggerPressed)) {
            shooterServoSubsystem.goToShooting();
        }

        // Turret (operator, gamepad2). The same controls as TurretAlpha. The first one that is
        // true wins.

        // The turret turns at the same power every time. A full stick push is the whole power.
        double turretPower = TURRET_POWER_PERCENT / PERCENT_PER_ONE;

        // Stick RIGHT is a positive number on the controller. But turning RIGHT means going
        // clockwise, and clockwise is a NEGATIVE angle. So we flip the sign of the stick.
        double turretStick = -gamepad2.left_stick_x;
        boolean usingTurretStick = Math.abs(turretStick) > TURRET_STICK_DEADBAND;

        String turretControl;
        if (usingTurretStick) {
            // The stick scales the power: a full push is the whole turret power,
            // a half push is half of it.
            turretSubsystem.setManualPower(turretStick * turretPower);
            turretControl = "Left Stick (MANUAL)";
        } else if (wasTurningTurretManually) {
            // The stick was just let go, so hold the turret where it stopped.
            turretSubsystem.holdCurrentAngle();
            turretControl = "Released (HOLD)";
        } else if (yPressed) {
            turretSubsystem.setTargetAngle(TURRET_PRESET_CENTER_DEGREES);
            turretControl = "Y (preset CENTER)";
        } else if (xPressed) {
            turretSubsystem.setTargetAngle(TURRET_PRESET_LEFT_DEGREES);
            turretControl = "X (preset FAR LEFT)";
        } else if (bPressed) {
            turretSubsystem.setTargetAngle(TURRET_PRESET_RIGHT_DEGREES);
            turretControl = "B (preset FAR RIGHT)";
        } else if (aPressed) {
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
        wasTurningTurretManually = usingTurretStick;
        wasGateShootTriggerPressed = gateShootTriggerPressed;
        wasGateLoadTriggerPressed = gateLoadTriggerPressed;
        wasDpadLeftPressed = dpadLeftPressed;
        wasDpadRightPressed = dpadRightPressed;

        // 4. Show what is happening on the Driver Station, one section at a time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive Control", driveControl);
        telemetry.addData("Drive inputs", "forward %.2f   strafe %.2f   turn %.2f",
                forward, strafe, turn);

        // The dials: what the operator has asked for. (They only take effect while the driver
        // holds the Right Trigger.)
        telemetry.addLine("--- SHOOTER DIALS (RPM) ---");
        telemetry.addData("Flywheel target RPM", "%d", flywheelRpmDial);
        telemetry.addData("Ratio (hood / flywheel)", "%.2f", ratio);
        if (hoodWheelCapped) {
            telemetry.addData("Hood wheel target RPM", "%.0f  CAPPED at %.0f (you asked for %.0f)",
                    hoodWheelTargetRpm, maxTargetRpm, wantedHoodWheelRpm);
        } else {
            telemetry.addData("Hood wheel target RPM", "%.0f", hoodWheelTargetRpm);
        }
        telemetry.addData("Shooter switch", shooterControl);

        // The shooter prints its own "--- SHOOTER ---" heading, with its state, "At Speed?",
        // and each wheel's real RPM next to its target. This is the data from the encoders.
        shooterSubsystem.addTelemetry(telemetry);

        // Is it OK to shoot? This comes from the real wheel speeds, not from a timer.
        ShooterSubsystem.ShooterState shooterState = shooterSubsystem.getState();
        String readyText;
        if (shooterState == ShooterSubsystem.ShooterState.READY) {
            readyText = "YES - OK to shoot";
        } else if (shooterState == ShooterSubsystem.ShooterState.SPINNING_UP) {
            readyText = "WAIT - wheels are not at their target RPM yet";
        } else if (shooterState == ShooterSubsystem.ShooterState.OPEN_LOOP) {
            readyText = "NO - reversing (clearing a jam)";
        } else {
            readyText = "shooter is OFF (hold the Right Trigger to shoot)";
        }
        telemetry.addData("OK to shoot?", readyText);

        // The hood wheel RPM / flywheel RPM we really have right now. It should match the ratio
        // dial once the wheels are at speed. If it does not, the hood wheel may be CAPPED.
        if (shooterSubsystem.getFlywheelRpm() > MIN_RPM_FOR_MEASURED_RATIO) {
            telemetry.addData("Ratio measured now", "%.2f",
                    shooterSubsystem.getHoodWheelRpm() / shooterSubsystem.getFlywheelRpm());
        }

        // The gate prints its own "--- SHOOTER GATE ---" heading.
        shooterServoSubsystem.addTelemetry(telemetry);

        // The intake prints its own "--- INTAKE ---" heading, so we add our line under it.
        intakeSubsystem.addTelemetry(telemetry);
        telemetry.addData("Intake Control", intakeControl);

        // The turret prints its own "--- TURRET ---" heading, so we add our lines under it.
        turretSubsystem.addTelemetry(telemetry);
        telemetry.addData("Turret Control", turretControl);
        telemetry.addData("Turret power", "%d%%", TURRET_POWER_PERCENT);
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
