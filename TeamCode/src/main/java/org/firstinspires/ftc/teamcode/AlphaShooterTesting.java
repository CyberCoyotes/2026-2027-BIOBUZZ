package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Alpha-Shooter Testing
 * An iterative OpMode that runs the shooter FLYWHEEL and the BACKSPIN wheel
 * (goBILDA 5200 series) using gamepad1. This is the shooter part of
 * AlphaCombinedTesting, on its own, so the shooter can be tested alone.
 *
 * HOLD a button to run BOTH the flywheel and the backspin at the SAME power:
 *   Y  -> 60% power
 *   X  -> 55% power
 *   B  -> 50% power
 *   A  -> 45% power
 *   No button -> both coast to a stop
 *
 * REVERSE (for clearing a jam, 60% power the other way):
 *   Right Bumper -> reverses the FLYWHEEL only
 *   Left Bumper  -> reverses the BACKSPIN only
 *
 * The backspin wheel spins the OPPOSITE way from the flywheel. That is set
 * in one place: FLYWHEEL_DIRECTION below. The backspin direction follows it.
 * A reverse bumper beats the shooter buttons for that motor.
 * If more than one shooter button is held, the HIGHEST power wins.
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: flywheel_motor   (goBILDA 5202/3/4 series)
 *   Motor name: backspin_motor   (goBILDA 5202/3/4 series)
 */
@TeleOp(name = "Alpha-Shooter Testing", group = "Testing")
public class AlphaShooterTesting extends OpMode {

    // Power for each button (0.0 to 1.0). The flywheel AND the backspin both
    // get this same power. Change these to test different speeds.
    private static final double SHOOTER_POWER_Y = 0.60;
    private static final double SHOOTER_POWER_X = 0.55;
    private static final double SHOOTER_POWER_B = 0.50;
    private static final double SHOOTER_POWER_A = 0.45;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double FLYWHEEL_POWER_REVERSE = -0.60;
    private static final double BACKSPIN_POWER_REVERSE = -0.60;

    // The flywheel direction. If the FLYWHEEL spins the wrong way, flip this
    // ONE line. The backspin wheel is always set to the opposite direction.
    private static final DcMotor.Direction FLYWHEEL_DIRECTION = DcMotor.Direction.FORWARD;

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
    private DcMotorEx backspinMotor = null;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- SHOOTER (flywheel + backspin) ---");
        telemetry.addLine("Y 60% | X 55% | B 50% | A 45%");
        telemetry.addLine("Right Bumper = REVERSE flywheel only");
        telemetry.addLine("Left Bumper = REVERSE backspin only");
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The names MUST match the robot configuration exactly.
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel_motor");
        backspinMotor = hardwareMap.get(DcMotorEx.class, "backspin_motor");

        // The flywheel uses FLYWHEEL_DIRECTION (set at the top of the file).
        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);

        // The backspin wheel spins the OPPOSITE way from the flywheel.
        if (FLYWHEEL_DIRECTION == DcMotor.Direction.FORWARD) {
            backspinMotor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            backspinMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        // We are controlling power directly. The encoder still reports speed for telemetry.
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backspinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // FLOAT lets the flywheel coast down. Do NOT use BRAKE on a heavy flywheel:
        // stopping it suddenly puts a lot of stress on the motor and gears.
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backspinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Display initialization status on Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Encoder Status", "NOT CONNECTED - Power control only");
        telemetry.addData("Flywheel Direction", FLYWHEEL_DIRECTION);
        telemetry.addData("Backspin Direction", backspinMotor.getDirection());
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
        addControlsTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double shooterPower;
        String shooterControl;

        // Check the shooter buttons from HIGHEST power to LOWEST.
        // The first one that is pressed wins, and the rest are skipped.
        if (gamepad1.y) {
            shooterPower = SHOOTER_POWER_Y;
            shooterControl = "Y";
        } else if (gamepad1.x) {
            shooterPower = SHOOTER_POWER_X;
            shooterControl = "X";
        } else if (gamepad1.b) {
            shooterPower = SHOOTER_POWER_B;
            shooterControl = "B";
        } else if (gamepad1.a) {
            shooterPower = SHOOTER_POWER_A;
            shooterControl = "A";
        } else {
            // No button held, so stop both motors.
            shooterPower = 0.0;
            shooterControl = "None";
        }

        // Both motors start with the same power.
        double flywheelPower = shooterPower;
        String flywheelControl = shooterControl;
        double backspinPower = shooterPower;
        String backspinControl = shooterControl;

        // REVERSE beats the shooter buttons, so clearing a jam always works.
        if (gamepad1.right_bumper) {
            flywheelPower = FLYWHEEL_POWER_REVERSE;
            flywheelControl = "Right Bumper (REVERSE)";
        }
        if (gamepad1.left_bumper) {
            backspinPower = BACKSPIN_POWER_REVERSE;
            backspinControl = "Left Bumper (REVERSE)";
        }

        // Send the power to the motors.
        flywheelMotor.setPower(flywheelPower);
        backspinMotor.setPower(backspinPower);

        // ENCODER NOT CONNECTED on the prototype. Remove the // when it is plugged in.
        // getVelocity() returns encoder ticks per second.
        // Convert to RPM: (ticks per second / ticks per rev) * 60 seconds.
        // double flywheelRpm = flywheelMotor.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
        // double backspinRpm = backspinMotor.getVelocity() / BACKSPIN_TICKS_PER_REV * 60.0;

        // Show what is happening on the Driver Station.
        // The power shown is the power we COMMANDED, so it is negative when reversing.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Flywheel Control", flywheelControl);
        telemetry.addData("Flywheel Power", "%.0f%%", flywheelPower * 100);
        telemetry.addData("Backspin Control", backspinControl);
        telemetry.addData("Backspin Power", "%.0f%%", backspinPower * 100);
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
        backspinMotor.setPower(0.0);
    }
}
