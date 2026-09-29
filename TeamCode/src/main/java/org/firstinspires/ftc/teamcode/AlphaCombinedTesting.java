package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Alpha-Combined Testing
 * An iterative OpMode that runs the INTAKE and the shooter FLYWHEEL together
 * using gamepad1. It combines AlphaIntakeTesting and AlphaShooterTesting.
 *
 * INTAKE (bumpers and face buttons) - HOLD a button to run the intake:
 *   Left Bumper  -> REVERSE at 60% power (push a stuck game piece back out)
 *   Right Bumper -> 100% power
 *   Y            ->  85% power
 *   X            ->  75% power
 *   A            ->  65% power
 *   No button    -> intake stops
 *
 * SHOOTER (triggers and D-Pad):
 *   Right Trigger (held)         -> flywheel runs at the DEFAULT power (100%)
 *   Right Trigger + D-Pad Up     -> 100% power
 *   Right Trigger + D-Pad Right  ->  85% power
 *   Right Trigger + D-Pad Left   ->  75% power
 *   Right Trigger + D-Pad Down   ->  65% power
 *   Left Trigger (held)          -> flywheel REVERSE at 60% power
 *   Nothing held                 -> flywheel coasts to a stop
 *
 * The intake and the shooter are separate. Each one has its own reverse:
 * Left BUMPER reverses the INTAKE, Left TRIGGER reverses the FLYWHEEL.
 * You can run both at the same time.
 *
 * On a PS5 controller: A = Cross, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: intake_motor     (goBILDA 5202/3/4 series)
 *   Motor name: flywheel_motor   (goBILDA 5202/3/4 series)
 */
@TeleOp(name = "Alpha-Combined Testing", group = "Testing")
public class AlphaCombinedTesting extends OpMode {

    // ---------- INTAKE settings ----------

    // Power for each intake button (0.0 to 1.0). Change these to test different speeds.
    private static final double INTAKE_POWER_RIGHT_BUMPER = 1.00;
    private static final double INTAKE_POWER_Y            = 0.85;
    private static final double INTAKE_POWER_X            = 0.75;
    private static final double INTAKE_POWER_A            = 0.65;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double INTAKE_POWER_REVERSE      = -0.60;

    // ---------- SHOOTER settings ----------

    // A trigger reads 0.0 (not pressed) to 1.0 (pressed all the way).
    // We count it as "held" once it is pushed past this number, so a light
    // accidental touch does not start the flywheel. Raise it if that happens.
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.10;

    // Power for each D-Pad direction (0.0 to 1.0). Change these to test different speeds.
    private static final double SHOOTER_POWER_DPAD_UP    = 1.00;
    private static final double SHOOTER_POWER_DPAD_RIGHT = 0.85;
    private static final double SHOOTER_POWER_DPAD_LEFT  = 0.75;
    private static final double SHOOTER_POWER_DPAD_DOWN  = 0.65;

    // Power when the Right Trigger is held and NO D-Pad direction is held.
    private static final double SHOOTER_POWER_DEFAULT    = 1.00;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double SHOOTER_POWER_REVERSE    = -0.60;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor = null;

    // DcMotorEx is a DcMotor with extra features, like reading speed (velocity).
    private DcMotorEx flywheelMotor = null;

    // These are filled in by updateIntake() and updateShooter() each loop,
    // so loop() can show them in telemetry.
    private double intakePower = 0.0;
    private String intakeControl = "None";
    private double flywheelPower = 0.0;
    private String shooterControl = "None";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The names "intake_motor" and "flywheel_motor" MUST match the robot configuration exactly.
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel_motor");

        // If a motor spins the wrong way, flip it between FORWARD and REVERSE.
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);

        // We are controlling power directly, not using the encoder for speed control.
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // FLOAT lets the motors coast to a stop.
        // Do NOT use BRAKE on the flywheel: stopping a heavy flywheel suddenly
        // puts a lot of stress on the motor and gears.
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Encoder Status", "NOT CONNECTED - Power control only");
        telemetry.addLine("Intake: bumpers + Y/X/A.  Shooter: Right Trigger + D-Pad.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
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
        // 1. Read the gamepad and decide what each mechanism should do.
        updateIntake();
        updateShooter();

        // 2. Send the power to the motors.
        intakeMotor.setPower(intakePower);
        flywheelMotor.setPower(flywheelPower);

        // 3. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Intake Control", intakeControl);
        telemetry.addData("Intake Power", "%.0f%%", intakePower * 100);
        telemetry.addData("Shooter Control", shooterControl);
        telemetry.addData("Flywheel Power", "%.0f%%", flywheelPower * 100);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        intakeMotor.setPower(0.0);
        flywheelMotor.setPower(0.0);
    }

    /*
     * Picks the intake power from the bumpers and Y / X / A.
     */
    private void updateIntake() {
        // Check REVERSE first, so clearing a jam always works.
        // Then check the intake buttons from HIGHEST power to LOWEST.
        // The first one that is pressed wins, and the rest are skipped.
        if (gamepad1.left_bumper) {
            intakePower = INTAKE_POWER_REVERSE;
            intakeControl = "Left Bumper (REVERSE)";
        } else if (gamepad1.right_bumper) {
            intakePower = INTAKE_POWER_RIGHT_BUMPER;
            intakeControl = "Right Bumper";
        } else if (gamepad1.y) {
            intakePower = INTAKE_POWER_Y;
            intakeControl = "Y";
        } else if (gamepad1.x) {
            intakePower = INTAKE_POWER_X;
            intakeControl = "X";
        } else if (gamepad1.a) {
            intakePower = INTAKE_POWER_A;
            intakeControl = "A";
        } else {
            // No button held, so stop the intake.
            intakePower = 0.0;
            intakeControl = "None";
        }
    }

    /*
     * Picks the flywheel power from the triggers and the D-Pad.
     */
    private void updateShooter() {
        boolean rightTriggerHeld = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean leftTriggerHeld  = gamepad1.left_trigger  > TRIGGER_PRESSED_THRESHOLD;

        // Check REVERSE first, so clearing a jam always works.
        if (leftTriggerHeld) {
            flywheelPower = SHOOTER_POWER_REVERSE;
            shooterControl = "Left Trigger (REVERSE)";
        } else if (rightTriggerHeld) {
            // The Right Trigger is held, so the flywheel runs.
            // The D-Pad picks the power. Check from HIGHEST power to LOWEST.
            if (gamepad1.dpad_up) {
                flywheelPower = SHOOTER_POWER_DPAD_UP;
                shooterControl = "Right Trigger + D-Pad Up";
            } else if (gamepad1.dpad_right) {
                flywheelPower = SHOOTER_POWER_DPAD_RIGHT;
                shooterControl = "Right Trigger + D-Pad Right";
            } else if (gamepad1.dpad_left) {
                flywheelPower = SHOOTER_POWER_DPAD_LEFT;
                shooterControl = "Right Trigger + D-Pad Left";
            } else if (gamepad1.dpad_down) {
                flywheelPower = SHOOTER_POWER_DPAD_DOWN;
                shooterControl = "Right Trigger + D-Pad Down";
            } else {
                flywheelPower = SHOOTER_POWER_DEFAULT;
                shooterControl = "Right Trigger (default power)";
            }
        } else {
            // Nothing held, so stop the flywheel.
            // The D-Pad alone does nothing on purpose.
            flywheelPower = 0.0;
            shooterControl = "None";
        }
    }
}
