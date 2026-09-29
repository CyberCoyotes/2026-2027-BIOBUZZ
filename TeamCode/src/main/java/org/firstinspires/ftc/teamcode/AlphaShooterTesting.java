package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Alpha-Shooter Testing
 * An iterative OpMode that runs the shooter FLYWHEEL motor (goBILDA 5200 series)
 * using gamepad1. The RIGHT TRIGGER is the "run the flywheel" control.
 *
 *   Right Trigger (held)            -> flywheel runs at the DEFAULT power (100%)
 *   Right Trigger + D-Pad Up        -> 100% power
 *   Right Trigger + D-Pad Right     ->  85% power
 *   Right Trigger + D-Pad Left      ->  75% power
 *   Right Trigger + D-Pad Down      ->  65% power
 *   Left Trigger (held)             -> REVERSE at 60% power (push a stuck game piece back out)
 *   Nothing held                    -> flywheel coasts to a stop
 *
 * The D-Pad only does something WHILE the Right Trigger is held.
 * Left Trigger beats everything else, so clearing a jam always works.
 * If more than one D-Pad direction is held, the HIGHEST power wins.
 *
 * BACKSPIN motor is commented out for now. Remove the // marks to turn it back on.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: flywheel_motor   (backspin_motor not needed yet)
 *   Motor type: goBILDA 5202/3/4 series
 */
@TeleOp(name = "Alpha-Shooter Testing", group = "Testing")
public class AlphaShooterTesting extends OpMode {

    // A trigger reads 0.0 (not pressed) to 1.0 (pressed all the way).
    // We count it as "held" once it is pushed past this number, so a light
    // accidental touch does not start the flywheel. Raise it if that happens.
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.10;

    // Power for each D-Pad direction (0.0 to 1.0). Change these to test different speeds.
    private static final double POWER_DPAD_UP    = 1.00;
    private static final double POWER_DPAD_RIGHT = 0.85;
    private static final double POWER_DPAD_LEFT  = 0.75;
    private static final double POWER_DPAD_DOWN  = 0.65;

    // Power when the Right Trigger is held and NO D-Pad direction is held.
    private static final double POWER_DEFAULT    = 1.00;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double POWER_REVERSE    = -0.60;

    // Backspin preset powers (not used yet).
    // private static final double BACKSPIN_LOW         = 0.25;
    // private static final double BACKSPIN_MEDIUM_LOW  = 0.50;
    // private static final double BACKSPIN_MEDIUM_HIGH = 0.75;
    // private static final double BACKSPIN_HIGH        = 1.00;

    // ENCODER NOT CONNECTED on the prototype. Remove the // when it is plugged in.
    // Encoder ticks per ONE turn of the motor's output shaft.
    // goBILDA 5202/3/4 motors: 28 ticks per turn of the motor itself.
    // If your motor has a gearbox, multiply by the gear ratio
    // (example: a 3.7:1 gearbox -> 28 * 3.7 = 103.6).
    // private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    // private static final double BACKSPIN_TICKS_PER_REV = 28.0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // DcMotorEx is a DcMotor with extra features, like reading speed (velocity).
    private DcMotorEx flywheelMotor = null;
    // private DcMotorEx backspinMotor = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The name "flywheel_motor" MUST match the robot configuration exactly.
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel_motor");
        // backspinMotor = hardwareMap.get(DcMotorEx.class, "backspin_motor");

        // If the flywheel spins the wrong way, flip this between FORWARD and REVERSE.
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        // backspinMotor.setDirection(DcMotor.Direction.FORWARD);

        // We are controlling power directly. The encoder still reports speed for telemetry.
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // backspinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // FLOAT lets the flywheel coast down. Do NOT use BRAKE on a heavy flywheel:
        // stopping it suddenly puts a lot of stress on the motor and gears.
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // backspinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Display initialization status on Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Flywheel Motor", "Connected");
        telemetry.addData("Encoder Status", "NOT CONNECTED - Power control only");
        telemetry.addData("Control", "Hold Right Trigger, pick power with D-Pad");
        telemetry.addLine("Ready to start!");
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
        double flywheelPower;
        String activeControl;

        boolean rightTriggerHeld = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean leftTriggerHeld  = gamepad1.left_trigger  > TRIGGER_PRESSED_THRESHOLD;

        // Check REVERSE first, so clearing a jam always works.
        if (leftTriggerHeld) {
            flywheelPower = POWER_REVERSE;
            activeControl = "Left Trigger (REVERSE)";
        } else if (rightTriggerHeld) {
            // The Right Trigger is held, so the flywheel runs.
            // The D-Pad picks the power. Check from HIGHEST power to LOWEST.
            // The first one that is pressed wins, and the rest are skipped.
            if (gamepad1.dpad_up) {
                flywheelPower = POWER_DPAD_UP;
                activeControl = "Right Trigger + D-Pad Up";
            } else if (gamepad1.dpad_right) {
                flywheelPower = POWER_DPAD_RIGHT;
                activeControl = "Right Trigger + D-Pad Right";
            } else if (gamepad1.dpad_left) {
                flywheelPower = POWER_DPAD_LEFT;
                activeControl = "Right Trigger + D-Pad Left";
            } else if (gamepad1.dpad_down) {
                flywheelPower = POWER_DPAD_DOWN;
                activeControl = "Right Trigger + D-Pad Down";
            } else {
                flywheelPower = POWER_DEFAULT;
                activeControl = "Right Trigger (default power)";
            }
        } else {
            // Nothing held, so stop the flywheel.
            // The D-Pad alone does nothing on purpose.
            flywheelPower = 0.0;
            activeControl = "None";
        }

        // Send the power to the motor.
        flywheelMotor.setPower(flywheelPower);

        // ENCODER NOT CONNECTED on the prototype. Remove the // when it is plugged in.
        // getVelocity() returns encoder ticks per second.
        // Convert to RPM: (ticks per second / ticks per rev) * 60 seconds.
        // double flywheelRpm = flywheelMotor.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
        // double backspinRpm = backspinMotor.getVelocity() / BACKSPIN_TICKS_PER_REV * 60.0;

        // Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Control", activeControl);
        telemetry.addData("Flywheel Power", "%.0f%%", flywheelPower * 100);
        // telemetry.addData("Flywheel RPM", "%.0f", flywheelRpm);
        // telemetry.addData("Backspin RPM", "%.0f", backspinRpm);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        flywheelMotor.setPower(0.0);
        // backspinMotor.setPower(0.0);
    }
}
