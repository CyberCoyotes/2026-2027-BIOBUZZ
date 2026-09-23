package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Alpha-Intake Testing
 * An iterative OpMode that runs ONE intake motor (goBILDA 5200 series)
 * using the left analog stick on gamepad1.
 *
 *   Push left stick UP   -> intake pulls game pieces IN
 *   Pull left stick DOWN -> intake pushes game pieces OUT
 *   Let go of the stick  -> intake stops
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: intake_motor
 *   Motor type: goBILDA 5202/3/4 series
 */
@TeleOp(name = "Alpha-Intake Testing", group = "Testing")
public class AlphaIntakeTesting extends OpMode {

    // Stick values smaller than this are treated as zero.
    // This stops the motor from creeping when the stick is not perfectly centered.
    private static final double STICK_DEADBAND = 0.05;

    // Maximum power the intake is allowed to use (0.0 to 1.0).
    // Lower this while testing if the intake is too aggressive.
    private static final double INTAKE_MAX_POWER = 1.0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The name "intake_motor" MUST match the robot configuration exactly.
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        // If pushing the stick UP spins the intake the wrong way, change FORWARD to REVERSE.
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // We are controlling power directly, not using the encoder for speed control.
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // FLOAT lets the intake coast to a stop. Try BRAKE if you want it to stop instantly.
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
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
        // The joystick Y axis is NEGATIVE when pushed up, so we flip the sign.
        double stickInput = -gamepad1.left_stick_y;

        // Apply the deadband: tiny stick movements count as zero.
        if (Math.abs(stickInput) < STICK_DEADBAND) {
            stickInput = 0.0;
        }

        // Scale by the max power and keep the result between -1.0 and 1.0.
        double intakePower = Range.clip(stickInput * INTAKE_MAX_POWER, -1.0, 1.0);

        // Send the power to the motor.
        intakeMotor.setPower(intakePower);

        // Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Stick", "%.2f", stickInput);
        telemetry.addData("Intake Power", "%.2f", intakePower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motor stopped.
        intakeMotor.setPower(0.0);
    }
}