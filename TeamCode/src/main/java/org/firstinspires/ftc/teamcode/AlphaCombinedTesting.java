package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Alpha-Combined Testing
 * An iterative OpMode that runs the INTAKE, the shooter FLYWHEEL, and the
 * BACKSPIN wheel together. It combines AlphaIntakeTesting and AlphaShooterTesting.
 *
 * Two controllers are used:
 *   gamepad1 (driver)   -> INTAKE  (driving will be added here later)
 *   gamepad2 (operator) -> SHOOTER (flywheel and backspin together)
 *
 * On the Driver Station, pair the controllers with Start + A (gamepad1)
 * and Start + B (gamepad2).
 *
 * GAMEPAD 1 - INTAKE (HOLD a button to run):
 *   Left Bumper  -> REVERSE at 60% power (push a stuck game piece back out)
 *   Right Bumper -> 100% power
 *   Y            ->  85% power
 *   X            ->  75% power
 *   A            ->  65% power
 *   No button    -> intake stops
 *
 * GAMEPAD 2 - SHOOTER (HOLD a button to run BOTH the flywheel and the backspin
 * at the SAME power):
 *   Y  -> 60% power
 *   X  -> 55% power
 *   B  -> 50% power
 *   A  -> 45% power
 *   No button -> both coast to a stop
 *
 * GAMEPAD 2 - SHOOTER REVERSE (for clearing a jam, 60% power the other way):
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
 *   Motor name: intake_motor     (goBILDA 5202/3/4 series)
 *   Motor name: flywheel_motor   (goBILDA 5202/3/4 series)
 *   Motor name: backspin_motor   (goBILDA 5202/3/4 series)
 */
@TeleOp(name = "Alpha-Combined Testing", group = "Testing")
public class AlphaCombinedTesting extends OpMode {

    // ---------- INTAKE settings (gamepad1) ----------

    // Power for each intake button (0.0 to 1.0). Change these to test different speeds.
    private static final double INTAKE_POWER_RIGHT_BUMPER = 1.00;
    private static final double INTAKE_POWER_Y            = 0.85;
    private static final double INTAKE_POWER_X            = 0.75;
    private static final double INTAKE_POWER_A            = 0.65;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double INTAKE_POWER_REVERSE      = -0.60;

    // ---------- SHOOTER settings (gamepad2) ----------

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

    // ---------- Telemetry settings ----------

    // After START, the button map stays on the screen for this many seconds,
    // then it goes away so the live data is easier to read.
    private static final double CONTROLS_DISPLAY_SECONDS = 5.0;

    // ENCODER NOT CONNECTED on the prototype. Remove the // when it is plugged in.
    // Encoder ticks per ONE turn of the motor's output shaft.
    // goBILDA 5202/3/4 motors: 28 ticks per turn of the motor itself.
    // If your motor has a gearbox, multiply by the gear ratio
    // (example: a 3.7:1 gearbox -> 28 * 3.7 = 103.6).
    // private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    // private static final double BACKSPIN_TICKS_PER_REV = 28.0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor = null;

    // DcMotorEx is a DcMotor with extra features, like reading speed (velocity).
    private DcMotorEx flywheelMotor = null;
    private DcMotorEx backspinMotor = null;

    // These are filled in by the update methods each loop,
    // so loop() can show them in telemetry.
    private double intakePower = 0.0;
    private String intakeControl = "None";
    private double flywheelPower = 0.0;
    private String flywheelControl = "None";
    private double backspinPower = 0.0;
    private String backspinControl = "None";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The names MUST match the robot configuration exactly.
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel_motor");
        backspinMotor = hardwareMap.get(DcMotorEx.class, "backspin_motor");

        // If the intake spins the wrong way, flip it between FORWARD and REVERSE.
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // The flywheel uses FLYWHEEL_DIRECTION (set at the top of the file).
        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);

        // The backspin wheel spins the OPPOSITE way from the flywheel.
        if (FLYWHEEL_DIRECTION == DcMotor.Direction.FORWARD) {
            backspinMotor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            backspinMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        // We are controlling power directly, not using the encoder for speed control.
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backspinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // FLOAT lets the motors coast to a stop.
        // Do NOT use BRAKE on the flywheel: stopping a heavy flywheel suddenly
        // puts a lot of stress on the motor and gears.
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backspinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        // 1. Read the gamepads and decide what each mechanism should do.
        updateIntake();
        updateShooter();

        // 2. Send the power to the motors.
        intakeMotor.setPower(intakePower);
        flywheelMotor.setPower(flywheelPower);
        backspinMotor.setPower(backspinPower);

        // ENCODER NOT CONNECTED on the prototype. Remove the // when it is plugged in.
        // getVelocity() returns encoder ticks per second.
        // Convert to RPM: (ticks per second / ticks per rev) * 60 seconds.
        // double flywheelRpm = flywheelMotor.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
        // double backspinRpm = backspinMotor.getVelocity() / BACKSPIN_TICKS_PER_REV * 60.0;

        // 3. Show what is happening on the Driver Station.
        // The power shown is the power we COMMANDED, so it is negative when reversing.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Intake Control (gp1)", intakeControl);
        telemetry.addData("Intake Power", "%.0f%%", intakePower * 100);
        telemetry.addData("Flywheel Control (gp2)", flywheelControl);
        telemetry.addData("Flywheel Power", "%.0f%%", flywheelPower * 100);
        telemetry.addData("Backspin Control (gp2)", backspinControl);
        telemetry.addData("Backspin Power", "%.0f%%", backspinPower * 100);
        // telemetry.addData("Flywheel RPM", "%.0f", flywheelRpm);
        // telemetry.addData("Backspin RPM", "%.0f", backspinRpm);

        // For the first few seconds after START, keep the button map on the screen.
        if (runtime.seconds() < CONTROLS_DISPLAY_SECONDS) {
            addControlsTelemetry();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        intakeMotor.setPower(0.0);
        flywheelMotor.setPower(0.0);
        backspinMotor.setPower(0.0);
    }

    /*
     * Adds the button map to telemetry. Called from init, init_loop, start,
     * and the first few seconds of loop, so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- GAMEPAD 1: INTAKE ---");
        telemetry.addLine("Right Bumper 100% | Y 85% | X 75% | A 65%");
        telemetry.addLine("Left Bumper = REVERSE");
        telemetry.addLine("--- GAMEPAD 2: SHOOTER (flywheel + backspin) ---");
        telemetry.addLine("Y 60% | X 55% | B 50% | A 45%");
        telemetry.addLine("Right Bumper = REVERSE flywheel only");
        telemetry.addLine("Left Bumper = REVERSE backspin only");
    }

    /*
     * Picks the intake power from gamepad1's bumpers and Y / X / A.
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
     * Picks the flywheel AND backspin power from gamepad2's Y / X / B / A.
     * Both motors get the same power. The reverse bumpers can then override
     * one motor at a time.
     */
    private void updateShooter() {
        double shooterPower;
        String shooterControl;

        // Check the shooter buttons from HIGHEST power to LOWEST.
        // The first one that is pressed wins, and the rest are skipped.
        if (gamepad2.y) {
            shooterPower = SHOOTER_POWER_Y;
            shooterControl = "Y";
        } else if (gamepad2.x) {
            shooterPower = SHOOTER_POWER_X;
            shooterControl = "X";
        } else if (gamepad2.b) {
            shooterPower = SHOOTER_POWER_B;
            shooterControl = "B";
        } else if (gamepad2.a) {
            shooterPower = SHOOTER_POWER_A;
            shooterControl = "A";
        } else {
            // No button held, so stop both motors.
            shooterPower = 0.0;
            shooterControl = "None";
        }

        // Both motors start with the same power.
        flywheelPower = shooterPower;
        flywheelControl = shooterControl;
        backspinPower = shooterPower;
        backspinControl = shooterControl;

        // REVERSE beats the shooter buttons, so clearing a jam always works.
        if (gamepad2.right_bumper) {
            flywheelPower = FLYWHEEL_POWER_REVERSE;
            flywheelControl = "Right Bumper (REVERSE)";
        }
        if (gamepad2.left_bumper) {
            backspinPower = BACKSPIN_POWER_REVERSE;
            backspinControl = "Left Bumper (REVERSE)";
        }
    }
}
