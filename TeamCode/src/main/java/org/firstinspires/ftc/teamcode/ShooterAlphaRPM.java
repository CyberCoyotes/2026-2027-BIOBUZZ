package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/*
 * Shooter Testing
 * An iterative OpMode that tests the ShooterSubsystem using gamepad1.
 * This is the SPEED-CONTROL version of AlphaShooterTesting: the buttons ask for an RPM
 * instead of a power. (AlphaShooterTesting is still there for plain power tests.)
 *
 * HOLD a button to run BOTH the flywheel and the backspin at a target speed.
 * Each wheel has its OWN RPM for each button, so you can tune them separately.
 * Starting values (flywheel / backspin):
 *   Y  -> 3600 / 3600 RPM
 *   X  -> 3300 / 3300 RPM
 *   B  -> 3000 / 3000 RPM
 *   A  -> 2700 / 2700 RPM
 *   No button -> both coast to a stop
 * Watch the shooter State on the screen: SPINNING_UP, then READY when both wheels are at speed.
 *
 * CHECK THE ENCODER (do this first!):
 *   Keep the wheels clear of hands and balls, then HOLD the Right Trigger.
 *   That runs both motors at FULL power with no speed control.
 *   Read "ticks/sec" on the screen:
 *     about 2,800  -> 28 ticks per rev is right (what the code uses now)
 *     about 11,200 -> it is really 112. Change the two TICKS_PER_REV numbers in ShooterSubsystem.
 *   Also check that both RPM readings are POSITIVE. If one is negative, the
 *   encoder or the motor direction is backwards.
 *
 * REVERSE (for clearing a jam, 60% power the other way):
 *   Right Bumper -> reverses the FLYWHEEL only
 *   Left Bumper  -> reverses the BACKSPIN only
 *   (The other wheel stops while you do this.)
 *
 * Priority, highest first: bumpers (jam clearing), right trigger (encoder check),
 * Y / X / B / A (speed control), then nothing (stop).
 * If more than one of Y / X / B / A is held, the HIGHEST speed wins.
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: flywheel_motor   (goBILDA 5203 series, encoder plugged in)
 *   Motor name: backspin_motor   (goBILDA 5203 series, encoder plugged in)
 */
@TeleOp(name = "Shooter Testing", group = "Testing")
public class ShooterTesting extends OpMode {

    // Each button sets the FLYWHEEL and the BACKSPIN target speed in RPM.
    // They are separate numbers on purpose, so you can tune each wheel on its own.
    // These start at the Alpha test power percents times 6000 RPM. They are guesses:
    // change them to try different speeds. The shooter will not go above its MAX_TARGET_RPM.
    //
    //                                     Y        X        B        A
    private static final double FLYWHEEL_RPM_Y = 3600.0;
    private static final double FLYWHEEL_RPM_X = 3300.0;
    private static final double FLYWHEEL_RPM_B = 3000.0;
    private static final double FLYWHEEL_RPM_A = 2700.0;

    private static final double BACKSPIN_RPM_Y = 3600.0;
    private static final double BACKSPIN_RPM_X = 3300.0;
    private static final double BACKSPIN_RPM_B = 3000.0;
    private static final double BACKSPIN_RPM_A = 2700.0;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double FLYWHEEL_POWER_REVERSE = -0.60;
    private static final double BACKSPIN_POWER_REVERSE = -0.60;

    // Power for the encoder check (hold the Right Trigger). 1.0 is full power.
    private static final double ENCODER_CHECK_POWER = 1.0;

    // The triggers give a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ShooterSubsystem shooterSubsystem;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS (flywheel / backspin RPM) ---");
        telemetry.addLine(String.format("Y: %.0f / %.0f   X: %.0f / %.0f",
                FLYWHEEL_RPM_Y, BACKSPIN_RPM_Y,
                FLYWHEEL_RPM_X, BACKSPIN_RPM_X));
        telemetry.addLine(String.format("B: %.0f / %.0f   A: %.0f / %.0f",
                FLYWHEEL_RPM_B, BACKSPIN_RPM_B,
                FLYWHEEL_RPM_A, BACKSPIN_RPM_A));
        telemetry.addLine("Right Trigger = FULL POWER encoder check");
        telemetry.addLine("Right Bumper = REVERSE flywheel only");
        telemetry.addLine("Left Bumper = REVERSE backspin only");
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The subsystem gets its own motors from the hardware map.
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        // Display initialization status on Driver Station
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
        addControlsTelemetry();
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
        // 1. Read sensors. (The shooter reads its own encoders inside update(), below.)

        // 2. Read the gamepad and decide what to do.
        // Check from HIGHEST priority to LOWEST. The first one that is true wins.
        String shooterControl;
        boolean reverseFlywheel = gamepad1.right_bumper;
        boolean reverseBackspin = gamepad1.left_bumper;
        boolean encoderCheck = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;

        // 3. Tell the shooter what to do, then call update().
        if (reverseFlywheel || reverseBackspin) {
            // REVERSE beats everything, so clearing a jam always works.
            // A wheel that is NOT being reversed gets 0 power.
            double flywheelPower = 0.0;
            double backspinPower = 0.0;
            if (reverseFlywheel) {
                flywheelPower = FLYWHEEL_POWER_REVERSE;
            }
            if (reverseBackspin) {
                backspinPower = BACKSPIN_POWER_REVERSE;
            }
            shooterSubsystem.setPower(flywheelPower, backspinPower);
            shooterControl = "Bumper (REVERSE)";
        } else if (encoderCheck) {
            shooterSubsystem.setPower(ENCODER_CHECK_POWER, ENCODER_CHECK_POWER);
            shooterControl = "Right Trigger (FULL POWER encoder check)";
        } else if (gamepad1.y) {
            shooterSubsystem.setTargetRpms(FLYWHEEL_RPM_Y, BACKSPIN_RPM_Y);
            shooterControl = "Y";
        } else if (gamepad1.x) {
            shooterSubsystem.setTargetRpms(FLYWHEEL_RPM_X, BACKSPIN_RPM_X);
            shooterControl = "X";
        } else if (gamepad1.b) {
            shooterSubsystem.setTargetRpms(FLYWHEEL_RPM_B, BACKSPIN_RPM_B);
            shooterControl = "B";
        } else if (gamepad1.a) {
            shooterSubsystem.setTargetRpms(FLYWHEEL_RPM_A, BACKSPIN_RPM_A);
            shooterControl = "A";
        } else {
            // No button held, so stop both motors. They coast down.
            shooterSubsystem.stop();
            shooterControl = "None";
        }
        shooterSubsystem.update();

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Control", shooterControl);
        shooterSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        shooterSubsystem.stop();
    }
}
