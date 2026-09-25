package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Alpha-Shooter Testing
 * An iterative OpMode that runs the shooter FLYWHEEL motor (goBILDA 5200 series)
 * using buttons on gamepad1. HOLD a button to run the flywheel.
 *
 *   Left Bumper  -> REVERSE at 60% power (push a stuck game piece back out)
 *   Right Bumper -> 100% power
 *   Y            ->  85% power
 *   X            ->  75% power
 *   A            ->  65% power
 *   No button    -> flywheel coasts to a stop
 *
 * Left Bumper beats every other button.
 * If more than one flywheel button is held, the HIGHEST power wins.
 *
 * On a PS5 controller: A = Cross, X = Square, Y = Triangle.
 *
 * BACKSPIN motor is commented out for now. Remove the // marks to turn it back on.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: flywheel_motor   (backspin_motor not needed yet)
 *   Motor type: goBILDA 5202/3/4 series
 *     // Encoder ticks per ONE turn of the motor's output shaft.
    // goBILDA 5202/3/4 motors: 28 ticks per turn of the motor itself.
    // If your motor has a gearbox, multiply by the gear ratio
    // (example: a 3.7:1 gearbox -> 28 * 3.7 = 103.6).
    // ENCODER NOT CONNECTED on the prototype. Remove the // when it is plugged in.
    // private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    // private static final double BACKSPIN_TICKS_PER_REV = 28.0;
 */
@TeleOp(name = "Alpha-Shooter Testing", group = "Testing")
public class AlphaShooterTesting extends OpMode {

    // Power for each button (0.0 to 1.0). Change these to test different speeds.
    private static final double POWER_RIGHT_BUMPER = 1.00;
    private static final double POWER_Y            = 0.85;
    private static final double POWER_X            = 0.75;
    private static final double POWER_A            = 0.65;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double POWER_LEFT_BUMPER  = -0.60;

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
        telemetry.addData("Control", "Hold buttons to run flywheel");
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
        String activeButton;

        // Check REVERSE first, so clearing a jam always works.
        // Then check the flywheel buttons from HIGHEST power to LOWEST.
        // The first one that is pressed wins, and the rest are skipped.
        if (gamepad1.left_bumper) {
            flywheelPower = POWER_LEFT_BUMPER;
            activeButton = "Left Bumper (REVERSE)";
        } else if (gamepad1.right_bumper) {
            flywheelPower = POWER_RIGHT_BUMPER;
            activeButton = "Right Bumper";
        } else if (gamepad1.y) {
            flywheelPower = POWER_Y;
            activeButton = "Y";
        } else if (gamepad1.x) {
            flywheelPower = POWER_X;
            activeButton = "X";
        } else if (gamepad1.a) {
            flywheelPower = POWER_A;
            activeButton = "A";
        } else {
            // No button held, so stop the flywheel.
            flywheelPower = 0.0;
            activeButton = "None";
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
        telemetry.addData("Button", activeButton);
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