package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Alpha-Intake Testing
 * An iterative OpMode that runs ONE intake motor (goBILDA 5200 series)
 * using buttons on gamepad1. HOLD a button to run the intake.
 *
 *   Left Bumper  -> REVERSE at 60% power (push a stuck game piece back out)
 *   Right Bumper -> 100% power
 *   Y            ->  85% power
 *   X            ->  75% power
 *   A            ->  65% power
 *   No button    -> intake stops
 *
 * Left Bumper beats every other button.
 * If more than one intake button is held, the HIGHEST power wins.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: intake_motor
 *   Motor type: goBILDA 5202/3/4 series
 */
@TeleOp(name = "Alpha-Intake Testing", group = "Testing")
public class AlphaIntakeTesting extends OpMode {

    // Power for each button (0.0 to 1.0). Change these to test different speeds.
    private static final double POWER_RIGHT_BUMPER = 1.00;
    private static final double POWER_Y            = 0.85;
    private static final double POWER_X            = 0.75;
    private static final double POWER_A            = 0.65;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double POWER_LEFT_BUMPER  = -0.60;

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

        // If the intake spins the wrong way, change FORWARD to REVERSE.
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

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
        double intakePower;
        String activeButton;

        // Check REVERSE first, so clearing a jam always works.
        // Then check the intake buttons from HIGHEST power to LOWEST.
        // The first one that is pressed wins, and the rest are skipped.
        if (gamepad1.left_bumper) {
            intakePower = POWER_LEFT_BUMPER;
            activeButton = "Left Bumper (REVERSE)";
        } else if (gamepad1.right_bumper) {
            intakePower = POWER_RIGHT_BUMPER;
            activeButton = "Right Bumper";
        } else if (gamepad1.y) {
            intakePower = POWER_Y;
            activeButton = "Y";
        } else if (gamepad1.x) {
            intakePower = POWER_X;
            activeButton = "X";
        } else if (gamepad1.a) {
            intakePower = POWER_A;
            activeButton = "A";
        } else {
            // No button held, so stop the intake.
            intakePower = 0.0;
            activeButton = "None";
        }

        // Send the power to the motor.
        intakeMotor.setPower(intakePower);

        // Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Button", activeButton);
        telemetry.addData("Intake Power", "%.0f%%", intakePower * 100);
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