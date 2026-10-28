package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/*
 * Shooter Testing, Alpha 2
 * An iterative OpMode that tests the ShooterSubsystem, the shooter GATE, and the INTAKE.
 * The shooter and the gate are on gamepad1. The intake is on gamepad2.
 * This is the SPEED-CONTROL version of ShooterPowerAlpha: the buttons ask for an RPM
 * instead of a power. (ShooterPowerAlpha is still there for plain power tests.)
 *
 * WHAT IS NEW IN ALPHA 2
 *   - The gate has THREE positions now (CLOSED, OPENED, LIFTED), each with its own button.
 *   - The intake runs from gamepad2, so you can feed real balls into the spinning wheels.
 *
 * HOLD a button to run BOTH the flywheel and the hood wheel at a target speed.
 * Each wheel has its OWN RPM for each button, so you can tune them separately.
 * Starting values (flywheel / hood wheel):
 *   Y  -> 1400 / 2800 RPM   (flywheel 1400, hood wheel twice that)
 *   X  -> 1300 / 2600 RPM   (flywheel 1300, hood wheel twice that)
 *   No button -> both coast to a stop
 * Those two power settings were the closest shots in testing, so the other presets were dropped.
 * The flywheel is the goBILDA 1620 RPM motor now (the hood wheel is still the 6000 RPM one), so
 * the old 3600 / 3300 RPM flywheel presets are gone: that motor tops out at 1620 RPM, and the
 * shooter will not take it above 1458. 1300 is about the speed that looked right on the robot
 * (see BiobuzzTeleOp). The hood wheel is twice the flywheel, the best ratio found so far.
 * Watch the shooter State on the screen: SPINNING_UP, then READY when both wheels are at speed.
 *
 * CHECK THE ENCODER (do this first!):
 *   Keep the wheels clear of hands and balls, then HOLD the Right Trigger.
 *   That runs both motors at FULL power with no speed control.
 *   The tick counts in ShooterSubsystem come from the goBILDA spec sheets (28 for the hood wheel
 *   motor, 103.8 for the flywheel motor), so this is a sanity check for wiring and config
 *   mistakes, not a way to find the numbers. Both encoders sit before the gearbox, so both
 *   motors read about 2,800 "ticks/sec" at full power. Then read "RPM" on the screen:
 *     Hood wheel (6000 RPM motor):  about 6,000 RPM
 *     Flywheel (1620 RPM motor):    about 1,500 to 1,620 RPM
 *   A motor reading far off means check the cables and the motor type in the Driver Station
 *   config first, then ask the coach.
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
 * SHOOTER GATE (the servo), separate from the wheel buttons above. All on gamepad1:
 *   D-pad Left   -> gate CLOSED  (holds the balls back, so balls can be loaded into the shooter)
 *   D-pad Right  -> gate OPENED  (lets a ball pass into a ready flywheel and shooter)
 *   Left Trigger -> gate LIFTED  (lifts to push the LAST ball into the shooter)
 *   D-pad Up     -> nudge the gate's CURRENT position toward 1.0 (small step)
 *   D-pad Down   -> nudge the gate's CURRENT position toward 0.0 (small step)
 * Each button moves the gate once per press. The three gate buttons are all on the LEFT side
 * of the controller, so your right thumb can keep holding Y or X for the wheels.
 * To tune a position, go to that state, then creep it with the D-pad Up / Down nudge. Read the
 * number on the screen, and then type it into CLOSED_POSITION / OPENED_POSITION /
 * LIFTED_POSITION in ShooterServoSubsystem. (The nudge only lasts until the OpMode stops.)
 * The gate starts CLOSED when you hit INIT. It does NOT depend on the wheels.
 *
 * INTAKE (gamepad2), so you can feed balls into the wheels:
 *   Right Bumper -> INTAKE (pull balls in)     Left Bumper -> REVERSE (push balls out)
 *   The intake runs at the power set in IntakeSubsystem (DEFAULT_INTAKE_POWER).
 *   Left Bumper beats Right Bumper, so clearing a jam always works.
 *   KEEP HANDS AND BALLS CLEAR OF THE WHEELS when you run the intake and shooter together.
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: flywheel   (goBILDA 5203 series, encoder plugged in)
 *   Motor name: hood_wheel   (goBILDA 5203 series, encoder plugged in)
 *   Motor name: intake   (IntakeSubsystem)
 *   Servo name: shooter_servo    (goBILDA dual mode servo, set to POSITIONAL mode)
 */
@TeleOp(name = "Shooter RPM - Alpha 2", group = "Testing")
public class ShooterRPMAlpha extends OpMode {

    // Each button sets the FLYWHEEL and the HOOD WHEEL target speed in RPM.
    // They are separate numbers on purpose, so you can tune each wheel on its own.
    // The flywheel motor is the goBILDA 1620 RPM one, so its numbers are small: the shooter will
    // not take it above FLYWHEEL_MAX_TARGET_RPM (1458). 1300 is about the speed that looked right
    // on the robot (see BiobuzzTeleOp). Change these after you test.
    //
    //                                     Y        X
    private static final double FLYWHEEL_RPM_Y = 1400.0;
    private static final double FLYWHEEL_RPM_X = 1300.0;

    // The hood wheel is TWICE the flywheel here (the best ratio found so far). Ball spin comes from the
    // DIFFERENCE between how fast the two wheel surfaces move (surface speed = RPM x diameter).
    // The flywheel is about 4 inches and the hood wheel about 2 inches, so at the SAME RPM
    // the flywheel surface moves about TWICE as fast. For a no-spin shot, the hood wheel RPM
    // needs to be about DOUBLE the flywheel RPM (for example 1300 flywheel with 2600 hood wheel).
    // To reduce spin, raise the hood wheel or lower the flywheel. Lowering the hood wheel makes
    // the spin worse.
    private static final double HOOD_WHEEL_RPM_Y = 2800.0;
    private static final double HOOD_WHEEL_RPM_X = 2600.0;

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

    // true = the control map stays on the Driver Station screen while the OpMode RUNS (at the
    // bottom, under the live data). false = it is only shown before START, like it used to be.
    private static final boolean SHOW_CONTROLS_WHILE_RUNNING = true;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private IntakeSubsystem intakeSubsystem;

    // What the gate buttons were doing last loop. We compare to this loop so that
    // holding a button gives ONE action, not a new one every loop (about 50 times a second).
    private boolean wasGateClosedPressed = false;
    private boolean wasGateOpenedPressed = false;
    private boolean wasGateLiftedPressed = false;
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
        telemetry.addLine("--- GATE (gamepad1) ---");
        telemetry.addLine("D-pad Left = CLOSED   D-pad Right = OPENED   Left Trigger = LIFTED");
        telemetry.addLine("D-pad Up / Down = nudge the current gate position +/-");
        telemetry.addLine("--- INTAKE (gamepad2) ---");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The subsystem gets its own motors from the hardware map.
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        // The gate servo moves to CLOSED here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

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
        intakeSubsystem.update();

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
        // to "pressed". Without this check, holding it would send the gate every loop.
        boolean gateClosedPressed = gamepad1.dpad_left;
        boolean gateOpenedPressed = gamepad1.dpad_right;
        boolean gateLiftedPressed = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean nudgeUpPressed = gamepad1.dpad_up;
        boolean nudgeDownPressed = gamepad1.dpad_down;

        if (gateClosedPressed && !wasGateClosedPressed) {
            shooterServoSubsystem.goToClosed();
        }
        if (gateOpenedPressed && !wasGateOpenedPressed) {
            shooterServoSubsystem.goToOpened();
        }
        if (gateLiftedPressed && !wasGateLiftedPressed) {
            shooterServoSubsystem.goToLifted();
        }
        if (nudgeUpPressed && !wasNudgeUpPressed) {
            shooterServoSubsystem.nudgePosition(GATE_NUDGE_STEP);
        }
        if (nudgeDownPressed && !wasNudgeDownPressed) {
            shooterServoSubsystem.nudgePosition(-GATE_NUDGE_STEP);
        }

        // Intake (gamepad2). Check REVERSE first, so clearing a jam always works.
        String intakeControl;
        if (gamepad2.left_bumper) {
            intakeSubsystem.reverse();
            intakeControl = "Left Bumper (REVERSE)";
        } else if (gamepad2.right_bumper) {
            intakeSubsystem.intake();
            intakeControl = "Right Bumper (INTAKE)";
        } else {
            // No bumper held, so stop the intake.
            intakeSubsystem.stop();
            intakeControl = "None";
        }

        // Remember this loop's buttons for the next loop.
        wasGateClosedPressed = gateClosedPressed;
        wasGateOpenedPressed = gateOpenedPressed;
        wasGateLiftedPressed = gateLiftedPressed;
        wasNudgeUpPressed = nudgeUpPressed;
        wasNudgeDownPressed = nudgeDownPressed;

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Control", shooterControl);
        shooterSubsystem.addTelemetry(telemetry);
        shooterServoSubsystem.addTelemetry(telemetry);

        // The intake prints its own "--- INTAKE ---" heading, so we add our line under it.
        intakeSubsystem.addTelemetry(telemetry);
        telemetry.addData("Intake Control", intakeControl);

        // 5. Keep the button map on the screen while running, at the bottom under the live data.
        //    (Before START the same map is shown by init() and init_loop().)
        if (SHOW_CONTROLS_WHILE_RUNNING) {
            addControlsTelemetry();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        // The gate servo just stays where it is. A servo holds its position by itself.
        shooterSubsystem.stop();
        intakeSubsystem.stop();
    }
}
