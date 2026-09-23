package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Alpha-Shooter Testing
 * An iterative OpMode that runs TWO shooter motors (goBILDA 5200 series):
 *   - a FLYWHEEL that launches the ball
 *   - a BACKSPIN motor that puts spin on the ball as it leaves
 *
 * Controls (gamepad1):
 *   Right trigger -> flywheel power (squeeze harder = faster)
 *
 *   Backspin presets (the button stays selected until you pick another):
 *     A (bottom)  -> LOW
 *     X (left)    -> MEDIUM-LOW
 *     B (right)   -> MEDIUM-HIGH
 *     Y (top)     -> HIGH
 *     Left bumper -> backspin OFF
 *
 *   On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station):
 *   Motor names: flywheel_motor, backspin_motor
 *   Motor type:  goBILDA 5202/3/4 series
 */
@TeleOp(name = "Alpha-Shooter Testing", group = "Testing")
public class AlphaShooterTesting extends OpMode {

    // Trigger values smaller than this are treated as zero.
    private static final double TRIGGER_DEADBAND = 0.05;

    // Maximum power the flywheel is allowed to use (0.0 to 1.0).
    private static final double FLYWHEEL_MAX_POWER = 1.0;

    // Backspin preset powers. Change these numbers while tuning your shot!
    private static final double BACKSPIN_LOW         = 0.25;
    private static final double BACKSPIN_MEDIUM_LOW  = 0.50;
    private static final double BACKSPIN_MEDIUM_HIGH = 0.75;
    private static final double BACKSPIN_HIGH        = 1.00;

    // Encoder ticks per ONE turn of the motor's output shaft.
    // goBILDA 5202/3/4 motors: 28 ticks per turn of the motor itself.
    // If your motor has a gearbox, multiply by the gear ratio
    // (example: a 3.7:1 gearbox -> 28 * 3.7 = 103.6).
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double BACKSPIN_TICKS_PER_REV = 28.0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // DcMotorEx is a DcMotor with extra features, like reading speed (velocity).
    private DcMotorEx flywheelMotor = null;
    private DcMotorEx backspinMotor = null;

    // The backspin power the driver has selected. It stays until a new button is pressed.
    private double backspinPower = 0.0;
    private String backspinPresetName = "OFF";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // These names MUST match the robot configuration exactly.
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel_motor");
        backspinMotor = hardwareMap.get(DcMotorEx.class, "backspin_motor");

        // If a motor spins the wrong way, change FORWARD to REVERSE.
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        backspinMotor.setDirection(DcMotor.Direction.FORWARD);

        // We are controlling power directly. The encoder still reports speed for telemetry.
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backspinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // FLOAT lets the wheels coast down. Do NOT use BRAKE on a heavy flywheel:
        // stopping it suddenly puts a lot of stress on the motor and gears.
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backspinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        // ---------- FLYWHEEL: analog trigger ----------
        // Triggers go from 0.0 (released) to 1.0 (fully squeezed).
        double triggerInput = gamepad1.right_trigger;

        if (triggerInput < TRIGGER_DEADBAND) {
            triggerInput = 0.0;
        }

        double flywheelPower = Range.clip(triggerInput * FLYWHEEL_MAX_POWER, 0.0, 1.0);
        flywheelMotor.setPower(flywheelPower);

        // ---------- BACKSPIN: button presets ----------
        // Only ONE preset can be chosen per loop. If two buttons are held,
        // the one checked first wins (this is how an if / else if chain works).
        if (gamepad1.left_bumper) {
            backspinPower = 0.0;
            backspinPresetName = "OFF";
        } else if (gamepad1.a) {
            backspinPower = BACKSPIN_LOW;
            backspinPresetName = "LOW (A)";
        } else if (gamepad1.x) {
            backspinPower = BACKSPIN_MEDIUM_LOW;
            backspinPresetName = "MEDIUM-LOW (X)";
        } else if (gamepad1.b) {
            backspinPower = BACKSPIN_MEDIUM_HIGH;
            backspinPresetName = "MEDIUM-HIGH (B)";
        } else if (gamepad1.y) {
            backspinPower = BACKSPIN_HIGH;
            backspinPresetName = "HIGH (Y)";
        }
        // If no button is pressed, backspinPower keeps its last value.

        backspinMotor.setPower(backspinPower);

        // ---------- TELEMETRY ----------
        // getVelocity() returns encoder ticks per second.
        // Convert to RPM: (ticks per second / ticks per rev) * 60 seconds.
        double flywheelRpm = flywheelMotor.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
        double backspinRpm = backspinMotor.getVelocity() / BACKSPIN_TICKS_PER_REV * 60.0;

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Flywheel Power", "%.2f", flywheelPower);
        telemetry.addData("Flywheel RPM", "%.0f", flywheelRpm);
        telemetry.addData("Backspin Preset", backspinPresetName);
        telemetry.addData("Backspin Power", "%.2f", backspinPower);
        telemetry.addData("Backspin RPM", "%.0f", backspinRpm);
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