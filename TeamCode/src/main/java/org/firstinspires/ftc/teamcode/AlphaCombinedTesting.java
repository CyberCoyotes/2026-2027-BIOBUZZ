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
 *   gamepad2 (operator) -> SHOOTER (flywheel and backspin, each on its own controls)
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
 * GAMEPAD 2 - FLYWHEEL (hold Right Trigger to run):
 *   Right Trigger                -> 100% power (the default)
 *   Right Trigger + D-Pad Up     -> 100% power
 *   Right Trigger + D-Pad Right  ->  85% power
 *   Right Trigger + D-Pad Left   ->  75% power
 *   Right Trigger + D-Pad Down   ->  65% power
 *   Right Bumper                 -> REVERSE at 60% power
 *
 * GAMEPAD 2 - BACKSPIN (hold Left Trigger to run):
 *   Left Trigger                 -> 100% power (the default)
 *   Left Trigger + Y             -> 100% power
 *   Left Trigger + B             ->  75% power
 *   Left Trigger + X             ->  50% power
 *   Left Trigger + A             ->  25% power
 *   Left Bumper                  -> REVERSE at 60% power
 *
 * The flywheel and the backspin are SEPARATE on purpose, so each one can be
 * tuned on its own. The D-Pad only changes the flywheel. The Y/B/X/A buttons
 * only change the backspin. You can run both at the same time.
 * Each motor's reverse beats its own power buttons.
 * If more than one preset button is held, the HIGHEST power wins.
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

    // A trigger reads 0.0 (not pressed) to 1.0 (pressed all the way).
    // We count it as "held" once it is pushed past this number, so a light
    // accidental touch does not start a motor. Raise it if that happens.
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.10;

    // FLYWHEEL power for each D-Pad direction (0.0 to 1.0).
    private static final double FLYWHEEL_POWER_DPAD_UP    = 1.00;
    private static final double FLYWHEEL_POWER_DPAD_RIGHT = 0.85;
    private static final double FLYWHEEL_POWER_DPAD_LEFT  = 0.75;
    private static final double FLYWHEEL_POWER_DPAD_DOWN  = 0.65;

    // Flywheel power when the Right Trigger is held and NO D-Pad direction is held.
    private static final double FLYWHEEL_POWER_DEFAULT    = 1.00;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double FLYWHEEL_POWER_REVERSE    = -0.60;

    // BACKSPIN power for each face button (0.0 to 1.0).
    private static final double BACKSPIN_POWER_HIGH        = 1.00;  // Y
    private static final double BACKSPIN_POWER_MEDIUM_HIGH = 0.75;  // B
    private static final double BACKSPIN_POWER_MEDIUM_LOW  = 0.50;  // X
    private static final double BACKSPIN_POWER_LOW         = 0.25;  // A

    // Backspin power when the Left Trigger is held and NO face button is held.
    private static final double BACKSPIN_POWER_DEFAULT     = 1.00;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double BACKSPIN_POWER_REVERSE     = -0.60;

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

        // If a motor spins the wrong way, flip it between FORWARD and REVERSE.
        // Both were flipped after testing on the robot:
        //   intake   was REVERSE, now FORWARD
        //   backspin was FORWARD, now REVERSE
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        backspinMotor.setDirection(DcMotor.Direction.REVERSE);

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
        updateFlywheel();
        updateBackspin();

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
        telemetry.addLine("--- GAMEPAD 2: FLYWHEEL ---");
        telemetry.addLine("Hold Right Trigger (100%) + D-Pad:");
        telemetry.addLine("Up 100% | Right 85% | Left 75% | Down 65%");
        telemetry.addLine("Right Bumper = REVERSE");
        telemetry.addLine("--- GAMEPAD 2: BACKSPIN ---");
        telemetry.addLine("Hold Left Trigger (100%) + button:");
        telemetry.addLine("Y 100% | B 75% | X 50% | A 25%");
        telemetry.addLine("Left Bumper = REVERSE");
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
     * Picks the flywheel power from gamepad2's Right Trigger, D-Pad, and Right Bumper.
     */
    private void updateFlywheel() {
        boolean rightTriggerHeld = gamepad2.right_trigger > TRIGGER_PRESSED_THRESHOLD;

        // Check REVERSE first, so clearing a jam always works.
        if (gamepad2.right_bumper) {
            flywheelPower = FLYWHEEL_POWER_REVERSE;
            flywheelControl = "Right Bumper (REVERSE)";
        } else if (rightTriggerHeld) {
            // The Right Trigger is held, so the flywheel runs.
            // The D-Pad picks the power. Check from HIGHEST power to LOWEST.
            if (gamepad2.dpad_up) {
                flywheelPower = FLYWHEEL_POWER_DPAD_UP;
                flywheelControl = "Right Trigger + D-Pad Up";
            } else if (gamepad2.dpad_right) {
                flywheelPower = FLYWHEEL_POWER_DPAD_RIGHT;
                flywheelControl = "Right Trigger + D-Pad Right";
            } else if (gamepad2.dpad_left) {
                flywheelPower = FLYWHEEL_POWER_DPAD_LEFT;
                flywheelControl = "Right Trigger + D-Pad Left";
            } else if (gamepad2.dpad_down) {
                flywheelPower = FLYWHEEL_POWER_DPAD_DOWN;
                flywheelControl = "Right Trigger + D-Pad Down";
            } else {
                flywheelPower = FLYWHEEL_POWER_DEFAULT;
                flywheelControl = "Right Trigger (default power)";
            }
        } else {
            // Nothing held, so stop the flywheel.
            // The D-Pad alone does nothing on purpose.
            flywheelPower = 0.0;
            flywheelControl = "None";
        }
    }

    /*
     * Picks the backspin power from gamepad2's Left Trigger, Y / B / X / A, and Left Bumper.
     */
    private void updateBackspin() {
        boolean leftTriggerHeld = gamepad2.left_trigger > TRIGGER_PRESSED_THRESHOLD;

        // Check REVERSE first, so clearing a jam always works.
        if (gamepad2.left_bumper) {
            backspinPower = BACKSPIN_POWER_REVERSE;
            backspinControl = "Left Bumper (REVERSE)";
        } else if (leftTriggerHeld) {
            // The Left Trigger is held, so the backspin wheel runs.
            // The face buttons pick the power. Check from HIGHEST power to LOWEST.
            if (gamepad2.y) {
                backspinPower = BACKSPIN_POWER_HIGH;
                backspinControl = "Left Trigger + Y";
            } else if (gamepad2.b) {
                backspinPower = BACKSPIN_POWER_MEDIUM_HIGH;
                backspinControl = "Left Trigger + B";
            } else if (gamepad2.x) {
                backspinPower = BACKSPIN_POWER_MEDIUM_LOW;
                backspinControl = "Left Trigger + X";
            } else if (gamepad2.a) {
                backspinPower = BACKSPIN_POWER_LOW;
                backspinControl = "Left Trigger + A";
            } else {
                backspinPower = BACKSPIN_POWER_DEFAULT;
                backspinControl = "Left Trigger (default power)";
            }
        } else {
            // Nothing held, so stop the backspin wheel.
            // The face buttons alone do nothing on purpose.
            backspinPower = 0.0;
            backspinControl = "None";
        }
    }
}
