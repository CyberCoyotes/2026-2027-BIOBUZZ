package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/*
 * Shooter Testing
 * An iterative OpMode that tests the ShooterSubsystem using gamepad1.
 * This is the SPEED-CONTROL version of ShooterPowerAlpha: the buttons ask for an RPM
 * instead of a power. (ShooterPowerAlpha is still there for plain power tests.)
 *
 * HOLD a button to run BOTH the flywheel and the hood wheel at a target speed.
 * Each wheel has its OWN RPM for each button, so you can tune them separately.
 * Starting values (flywheel / hood wheel):
 *   Y  -> 3600 / 3600 RPM   (the 60% power shot from ShooterPowerAlpha)
 *   X  -> 3300 / 3300 RPM   (the 55% power shot from ShooterPowerAlpha)
 *   No button -> both coast to a stop
 * Those two power settings were the closest shots in testing, so the other presets were dropped.
 * The RPM numbers are an ESTIMATE: power x 6000 RPM (the motor's no-load top speed).
 * A wheel running a shot is slower than that, so compare with the RPM you read in
 * ShooterPowerAlpha and use the measured number if it is different.
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
 *   Left Bumper  -> reverses the HOOD WHEEL only
 *   (The other wheel stops while you do this.)
 *
 * Priority, highest first: bumpers (jam clearing), right trigger (encoder check),
 * Y / X (speed control), then nothing (stop).
 * If both Y and X are held, the HIGHER speed (Y) wins.
 *
 * SHOOTER GATE (the servo), separate from the wheel buttons above:
 *   Left Trigger -> TOGGLES the gate between LOADING and SHOOTING (one flip per press)
 *   D-pad Up     -> nudge the gate's CURRENT position toward 1.0 (small step)
 *   D-pad Down   -> nudge the gate's CURRENT position toward 0.0 (small step)
 * The gate positions are not measured yet. Use the D-pad nudge to creep the gate to the
 * right spot for each state, read the number on the screen, and then type it into
 * LOADING_POSITION / SHOOTING_POSITION in ShooterServoSubsystem.
 * The gate starts in LOADING when you hit INIT. It does NOT depend on the wheels.
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: flywheel   (goBILDA 5203 series, encoder plugged in)
 *   Motor name: hood_wheel   (goBILDA 5203 series, encoder plugged in)
 *   Servo name: shooter_servo    (goBILDA dual mode servo, set to POSITIONAL mode)
 */
@TeleOp(name = "Shooter RPM - Alpha", group = "Testing")
public class ShooterRPMAlpha extends OpMode {

    // Each button sets the FLYWHEEL and the HOOD WHEEL target speed in RPM.
    // They are separate numbers on purpose, so you can tune each wheel on its own.
    // These are ESTIMATES worked out from the two power shots that tested best in
    // ShooterPowerAlpha: RPM = power x 6000 (the motor's top speed with no load).
    //   Y: 60% power -> 0.60 x 6000 = 3600 RPM
    //   X: 55% power -> 0.55 x 6000 = 3300 RPM
    // The real wheel runs slower than the no-load number when it is shooting, so check the
    // measured RPM in ShooterPowerAlpha and change these if they are different. The shooter
    // will not go above its MAX_TARGET_RPM.
    //
    //                                     Y        X
    private static final double FLYWHEEL_RPM_Y = 3600.0;
    private static final double FLYWHEEL_RPM_X = 3300.0;

    // The hood wheel matches the flywheel here, like the power test. Ball spin comes from the
    // DIFFERENCE between how fast the two wheel surfaces move (surface speed = RPM x diameter).
    // The flywheel is about 4 inches and the hood wheel about 2 inches, so at the SAME RPM
    // the flywheel surface moves about TWICE as fast. For a no-spin shot, the hood wheel RPM
    // needs to be about DOUBLE the flywheel RPM (for example 2700 flywheel with 5400 hood wheel).
    // To reduce spin, raise the hood wheel or lower the flywheel. Lowering the hood wheel makes
    // the spin worse.
    private static final double HOOD_WHEEL_RPM_Y = 3600.0;
    private static final double HOOD_WHEEL_RPM_X = 3300.0;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double FLYWHEEL_POWER_REVERSE = -0.60;
    private static final double HOOD_WHEEL_POWER_REVERSE = -0.60;

    // Power for the encoder check (hold the Right Trigger). 1.0 is full power.
    private static final double ENCODER_CHECK_POWER = 1.0;

    // The triggers give a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // How far one D-pad press moves the gate servo position (the position runs 0.0 to 1.0).
    // 0.01 is about 3 degrees on a 300 degree servo (0.01 x 300). Make it smaller for
    // finer adjustment, or bigger to move faster.
    private static final double GATE_NUDGE_STEP = 0.01;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;

    // What the gate buttons were doing last loop. We compare to this loop so that
    // holding a button gives ONE action, not a new one every loop (about 50 times a second).
    private boolean wasGateTogglePressed = false;
    private boolean wasNudgeUpPressed = false;
    private boolean wasNudgeDownPressed = false;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS (flywheel / hood wheel RPM) ---");
        telemetry.addLine(String.format("Y: %.0f / %.0f   X: %.0f / %.0f",
                FLYWHEEL_RPM_Y, HOOD_WHEEL_RPM_Y,
                FLYWHEEL_RPM_X, HOOD_WHEEL_RPM_X));
        telemetry.addLine("Right Trigger = FULL POWER encoder check");
        telemetry.addLine("Right Bumper = REVERSE flywheel only");
        telemetry.addLine("Left Bumper = REVERSE hood wheel only");
        telemetry.addLine("--- GATE ---");
        telemetry.addLine("Left Trigger = TOGGLE gate (loading / shooting)");
        telemetry.addLine("D-pad Up / Down = nudge gate position +/-");
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The subsystem gets its own motors from the hardware map.
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        // The gate servo moves to LOADING here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);

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
        boolean reverseHoodWheel = gamepad1.left_bumper;
        boolean encoderCheck = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;

        // 3. Tell the shooter what to do, then call update().
        if (reverseFlywheel || reverseHoodWheel) {
            // REVERSE beats everything, so clearing a jam always works.
            // A wheel that is NOT being reversed gets 0 power.
            double flywheelPower = 0.0;
            double hoodWheelPower = 0.0;
            if (reverseFlywheel) {
                flywheelPower = FLYWHEEL_POWER_REVERSE;
            }
            if (reverseHoodWheel) {
                hoodWheelPower = HOOD_WHEEL_POWER_REVERSE;
            }
            shooterSubsystem.setPower(flywheelPower, hoodWheelPower);
            shooterControl = "Bumper (REVERSE)";
        } else if (encoderCheck) {
            shooterSubsystem.setPower(ENCODER_CHECK_POWER, ENCODER_CHECK_POWER);
            shooterControl = "Right Trigger (FULL POWER encoder check)";
        } else if (gamepad1.y) {
            shooterSubsystem.setTargetRpms(FLYWHEEL_RPM_Y, HOOD_WHEEL_RPM_Y);
            shooterControl = "Y";
        } else if (gamepad1.x) {
            shooterSubsystem.setTargetRpms(FLYWHEEL_RPM_X, HOOD_WHEEL_RPM_X);
            shooterControl = "X";
        } else {
            // No button held, so stop both motors. They coast down.
            shooterSubsystem.stop();
            shooterControl = "None";
        }
        shooterSubsystem.update();

        // The gate is separate from the wheels, so it has its own buttons.
        // A button counts as ONE press only on the loop where it goes from "not pressed"
        // to "pressed". Without this check, holding it would flip the gate every loop.
        boolean gateTogglePressed = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean nudgeUpPressed = gamepad1.dpad_up;
        boolean nudgeDownPressed = gamepad1.dpad_down;

        if (gateTogglePressed && !wasGateTogglePressed) {
            shooterServoSubsystem.toggle();
        }
        if (nudgeUpPressed && !wasNudgeUpPressed) {
            shooterServoSubsystem.nudgePosition(GATE_NUDGE_STEP);
        }
        if (nudgeDownPressed && !wasNudgeDownPressed) {
            shooterServoSubsystem.nudgePosition(-GATE_NUDGE_STEP);
        }

        // Remember this loop's buttons for the next loop.
        wasGateTogglePressed = gateTogglePressed;
        wasNudgeUpPressed = nudgeUpPressed;
        wasNudgeDownPressed = nudgeDownPressed;

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Control", shooterControl);
        shooterSubsystem.addTelemetry(telemetry);
        shooterServoSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        // The gate servo just stays where it is. A servo holds its position by itself.
        shooterSubsystem.stop();
    }
}
