package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/*
 * Shooter Ratio Testing (POWER version)
 * An iterative OpMode for finding the best HOOD WHEEL RATIO, using plain POWER (a percent).
 * The printed sheet is TeamCode/docs/hood-wheel-ratio-test-sheet-power.html
 * (open it in a browser and print it).
 *
 * WHY POWER AND NOT RPM
 *   The encoder wires are not connected yet, so we cannot read or control wheel speed. This
 *   version sets POWER instead. Power is not exactly the same as speed: the real speed changes
 *   with the battery and with the ball squeezing the wheels. So use a FRESH battery, write down
 *   the battery volts, and treat the results as a first look.
 *   When the encoders are connected, the speed-control (RPM) version of this OpMode is saved in
 *   git: commit 89c074f. The RPM printed sheet is hood-wheel-ratio-test-sheet.html.
 *
 * THE IDEA
 *   RATIO = HOOD WHEEL POWER / FLYWHEEL POWER (a stand-in for the speed ratio). The hood wheel
 *   is smaller (about 2 inches) than the flywheel (about 4 inches), so a ratio near 2.0 should
 *   make the two wheel SURFACES move at the same speed and give the ball little or no spin.
 *   This test finds out what ratio really gives the least spin. When you find the best ratio,
 *   type it into HOOD_WHEEL_RATIO in ShooterSubsystem.
 *
 * THE TWO DIALS (you turn them with buttons, so you do not have to change code and redeploy)
 *   B           -> flywheel power UP   by FLYWHEEL_POWER_PERCENT_STEP
 *   A           -> flywheel power DOWN by FLYWHEEL_POWER_PERCENT_STEP
 *   D-pad Right -> ratio UP   by 0.05
 *   D-pad Left  -> ratio DOWN by 0.05
 *   The hood wheel power is always flywheel power x ratio. The screen shows all three.
 *   A wheel cannot go past 100% power. If the hood wheel power would be over 100% the screen
 *   says CAPPED. Do not record that shot: the real ratio is lower than the dial says.
 *   Example: 50% flywheel x ratio 2.00 = 100% hood wheel, the most it can do.
 *
 * GAMEPAD 1 (shooter and gate)
 *   Y            -> shooter ON (spins at the dialed powers and keeps spinning)
 *   X            -> shooter OFF (the wheels coast down. Wait for them to STOP before touching.)
 *   Right Bumper -> HOLD to reverse the flywheel only (clearing a jam)
 *   Left Bumper  -> HOLD to reverse the hood wheel only (clearing a jam)
 *   Left Trigger -> TOGGLES the gate between LOADING and SHOOTING (one flip per press)
 *   D-pad Up/Down-> nudge the gate position (the gate positions are still being measured)
 *
 * WHEELS SETTLED?
 *   With no encoder we cannot tell when the wheels are up to speed. So the screen counts
 *   SPIN_SETTLE_SECONDS after the shooter starts, after any dial change, and after every gate
 *   flip (the ball slows the wheels, and they need time to recover). Shoot when it says YES.
 *
 * GAMEPAD 2 (intake)
 *   Right Bumper -> INTAKE (pull balls in)     Left Bumper -> REVERSE
 *   The intake runs at the power set in IntakeSubsystem (DEFAULT_INTAKE_POWER).
 *
 * SAFE USE: keep hands, hair and loose clothing away from the wheels, wear eye protection, and
 * keep everyone out of the path of the ball.
 *
 * The shooter's own telemetry (RPM and ticks per second) is left out on purpose: with no
 * encoders it reads zero. Add shooterSubsystem.addTelemetry(telemetry) back when they are connected.
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station). The names live in the subsystem classes:
 *   Motor names: flywheel, hood_wheel       (ShooterSubsystem)
 *   Motor name:  intake                     (IntakeSubsystem)
 *   Servo name:  shooter_servo              (ShooterServoSubsystem)
 */
@TeleOp(name = "Shooter Ratio - Alpha", group = "Testing")
public class ShooterRatioAlpha extends OpMode {

    // ------------------------------------------------------------------
    // The two dials. These are test-only numbers.
    // ------------------------------------------------------------------
    // The flywheel power dial is counted in WHOLE PERCENT (30 means 30% power).
    // Where it starts, how far one press moves it, and how low and high it can go.
    private static final int FLYWHEEL_POWER_PERCENT_START = 30;
    private static final int FLYWHEEL_POWER_PERCENT_STEP = 5;
    private static final int FLYWHEEL_POWER_PERCENT_MIN = 5;
    private static final int FLYWHEEL_POWER_PERCENT_MAX = 100;

    // The ratio dial is counted in HUNDREDTHS so that adding 5 over and over never drifts to
    // something like 1.9999999. So 200 means a ratio of 2.00, and a step of 5 means 0.05.
    private static final int RATIO_HUNDREDTHS_START = 200;
    private static final int RATIO_HUNDREDTHS_STEP = 5;
    private static final int RATIO_HUNDREDTHS_MIN = 100;
    private static final int RATIO_HUNDREDTHS_MAX = 300;

    // How many hundredths make a whole number (200 hundredths = 2.00).
    private static final double HUNDREDTHS_PER_ONE = 100.0;

    // How many percent make a whole power of 1.0 (100 percent = 1.0 power).
    private static final double PERCENT_PER_ONE = 100.0;

    // ------------------------------------------------------------------
    // Other test numbers
    // ------------------------------------------------------------------
    // The most power a wheel can be given, in percent. A hood wheel power above this is CAPPED.
    private static final double MAX_POWER_PERCENT = 100.0;

    // How long the wheels need to settle after the shooter starts, a dial changes, or a ball
    // goes through. A starting guess. Time it with a stopwatch: how long until the wheels
    // sound steady?
    private static final double SPIN_SETTLE_SECONDS = 3.0;

    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double FLYWHEEL_POWER_REVERSE = -0.60;
    private static final double HOOD_WHEEL_POWER_REVERSE = -0.60;

    // The triggers give a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // How far one D-pad press moves the gate servo position (the position runs 0.0 to 1.0).
    // 0.01 is about 3 degrees on a 300 degree servo (0.01 x 300).
    private static final double GATE_NUDGE_STEP = 0.01;

    // ------------------------------------------------------------------
    // Declare OpMode members.
    // ------------------------------------------------------------------
    private ElapsedTime runtime = new ElapsedTime();
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private IntakeSubsystem intakeSubsystem;

    // The dials, in whole numbers.
    private int flywheelPowerPercent = FLYWHEEL_POWER_PERCENT_START;
    private int ratioHundredths = RATIO_HUNDREDTHS_START;

    // True after Y, false after X. The shooter keeps spinning between button presses.
    private boolean shooterOn = false;

    // Times how long since the wheels last had a reason to change speed.
    private ElapsedTime spinTimer = new ElapsedTime();

    // What the buttons were doing LAST loop. A press lasts many loops (about 50 per second),
    // but we want it to count only ONCE. So we only act when it was NOT pressed last loop
    // and IS pressed now.
    private boolean wasYPressed = false;
    private boolean wasXPressed = false;
    private boolean wasBPressed = false;
    private boolean wasAPressed = false;
    private boolean wasDpadLeftPressed = false;
    private boolean wasDpadRightPressed = false;
    private boolean wasGateTogglePressed = false;
    private boolean wasGateNudgeUpPressed = false;
    private boolean wasGateNudgeDownPressed = false;

    /*
     * True only on the loop where a button goes from "not pressed" to "pressed".
     */
    private boolean isNewPress(boolean pressedNow, boolean pressedLastLoop) {
        return pressedNow && !pressedLastLoop;
    }

    /*
     * Start the "wheels settled?" count over.
     */
    private void restartSpinTimer() {
        spinTimer.reset();
    }

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- GAMEPAD 1 ---");
        telemetry.addLine("Y = shooter ON   X = shooter OFF");
        telemetry.addLine("B / A = flywheel power up / down");
        telemetry.addLine("D-pad Right / Left = ratio up / down");
        telemetry.addLine("Left Trigger = TOGGLE gate   D-pad Up/Down = nudge gate");
        telemetry.addLine("Right / Left Bumper = reverse flywheel / hood wheel");
        telemetry.addLine("--- GAMEPAD 2 ---");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Each subsystem gets its own hardware from the hardware map.
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        // The gate servo moves to LOADING here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized (POWER control, no encoders)");
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

        // Show the gate here too, so you can check before START that it went to LOADING.
        shooterServoSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        restartSpinTimer();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors. (The shooter has no working encoders yet.)
        intakeSubsystem.update();

        // 2. Read the gamepads and decide what to do.
        boolean yPressed = gamepad1.y;
        boolean xPressed = gamepad1.x;
        boolean bPressed = gamepad1.b;
        boolean aPressed = gamepad1.a;
        boolean dpadLeftPressed = gamepad1.dpad_left;
        boolean dpadRightPressed = gamepad1.dpad_right;
        boolean gateTogglePressed = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean gateNudgeUpPressed = gamepad1.dpad_up;
        boolean gateNudgeDownPressed = gamepad1.dpad_down;

        // Shooter on and off. If both are pressed at once, OFF wins (the safer choice).
        if (isNewPress(yPressed, wasYPressed)) {
            shooterOn = true;
            restartSpinTimer();
        }
        if (isNewPress(xPressed, wasXPressed)) {
            shooterOn = false;
        }

        // Turn the dials. Any change makes the wheels change speed, so start the count over.
        if (isNewPress(bPressed, wasBPressed)) {
            flywheelPowerPercent = flywheelPowerPercent + FLYWHEEL_POWER_PERCENT_STEP;
            restartSpinTimer();
        }
        if (isNewPress(aPressed, wasAPressed)) {
            flywheelPowerPercent = flywheelPowerPercent - FLYWHEEL_POWER_PERCENT_STEP;
            restartSpinTimer();
        }
        if (isNewPress(dpadRightPressed, wasDpadRightPressed)) {
            ratioHundredths = ratioHundredths + RATIO_HUNDREDTHS_STEP;
            restartSpinTimer();
        }
        if (isNewPress(dpadLeftPressed, wasDpadLeftPressed)) {
            ratioHundredths = ratioHundredths - RATIO_HUNDREDTHS_STEP;
            restartSpinTimer();
        }

        // Keep the dials between their lowest and highest settings.
        if (flywheelPowerPercent > FLYWHEEL_POWER_PERCENT_MAX) {
            flywheelPowerPercent = FLYWHEEL_POWER_PERCENT_MAX;
        }
        if (flywheelPowerPercent < FLYWHEEL_POWER_PERCENT_MIN) {
            flywheelPowerPercent = FLYWHEEL_POWER_PERCENT_MIN;
        }
        if (ratioHundredths > RATIO_HUNDREDTHS_MAX) {
            ratioHundredths = RATIO_HUNDREDTHS_MAX;
        }
        if (ratioHundredths < RATIO_HUNDREDTHS_MIN) {
            ratioHundredths = RATIO_HUNDREDTHS_MIN;
        }

        // The hood wheel power is the flywheel power times the ratio. A wheel cannot go past
        // 100%, so if the answer is more than that we use 100% and warn that it was CAPPED.
        double ratio = ratioHundredths / HUNDREDTHS_PER_ONE;
        double wantedHoodWheelPercent = flywheelPowerPercent * ratio;
        boolean hoodWheelCapped = wantedHoodWheelPercent > MAX_POWER_PERCENT;
        double hoodWheelPercent = Math.min(wantedHoodWheelPercent, MAX_POWER_PERCENT);

        // Turn the percents into the 0.0 to 1.0 powers the subsystem wants (30 -> 0.30).
        double flywheelPower = flywheelPowerPercent / PERCENT_PER_ONE;
        double hoodWheelPower = hoodWheelPercent / PERCENT_PER_ONE;

        // 3. Tell the subsystems what to do, then call their update().

        // Shooter wheels. Reverse beats everything, so clearing a jam always works.
        String shooterControl;
        boolean reverseFlywheel = gamepad1.right_bumper;
        boolean reverseHoodWheel = gamepad1.left_bumper;
        if (reverseFlywheel || reverseHoodWheel) {
            // A wheel that is NOT being reversed gets 0 power.
            double reverseFlywheelPower = 0.0;
            double reverseHoodWheelPower = 0.0;
            if (reverseFlywheel) {
                reverseFlywheelPower = FLYWHEEL_POWER_REVERSE;
            }
            if (reverseHoodWheel) {
                reverseHoodWheelPower = HOOD_WHEEL_POWER_REVERSE;
            }
            shooterSubsystem.setPower(reverseFlywheelPower, reverseHoodWheelPower);
            shooterControl = "Bumper (REVERSE)";
            // The wheels need time to settle again after a reverse.
            restartSpinTimer();
        } else if (shooterOn) {
            shooterSubsystem.setPower(flywheelPower, hoodWheelPower);
            shooterControl = "ON";
        } else {
            shooterSubsystem.stop();
            shooterControl = "OFF";
        }
        shooterSubsystem.update();

        // Gate. Flipping it lets a ball go through, which slows the wheels, so every flip
        // also starts the "wheels settled?" count over.
        if (isNewPress(gateTogglePressed, wasGateTogglePressed)) {
            shooterServoSubsystem.toggle();
            restartSpinTimer();
        }
        if (isNewPress(gateNudgeUpPressed, wasGateNudgeUpPressed)) {
            shooterServoSubsystem.nudgePosition(GATE_NUDGE_STEP);
        }
        if (isNewPress(gateNudgeDownPressed, wasGateNudgeDownPressed)) {
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
        wasYPressed = yPressed;
        wasXPressed = xPressed;
        wasBPressed = bPressed;
        wasAPressed = aPressed;
        wasDpadLeftPressed = dpadLeftPressed;
        wasDpadRightPressed = dpadRightPressed;
        wasGateTogglePressed = gateTogglePressed;
        wasGateNudgeUpPressed = gateNudgeUpPressed;
        wasGateNudgeDownPressed = gateNudgeDownPressed;

        // 4. Show what is happening on the Driver Station, one section at a time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addLine("--- THE DIALS ---");
        telemetry.addData("Flywheel power", "%d%%", flywheelPowerPercent);
        telemetry.addData("Ratio (hood / flywheel)", "%.2f", ratio);
        if (hoodWheelCapped) {
            telemetry.addData("Hood wheel power", "%.0f%%  CAPPED (you asked for %.0f%%) - DO NOT RECORD",
                    hoodWheelPercent, wantedHoodWheelPercent);
        } else {
            telemetry.addData("Hood wheel power", "%.0f%%", hoodWheelPercent);
        }

        telemetry.addLine("--- SHOOTER ---");
        telemetry.addData("Shooter switch", shooterControl);
        String settledText;
        if (!shooterOn) {
            settledText = "shooter is OFF";
        } else if (spinTimer.seconds() >= SPIN_SETTLE_SECONDS) {
            settledText = "YES - OK to shoot";
        } else {
            settledText = String.format("WAIT (%.1f of %.0f seconds)",
                    spinTimer.seconds(), SPIN_SETTLE_SECONDS);
        }
        telemetry.addData("Wheels settled?", settledText);

        // The gate prints its own "--- SHOOTER GATE ---" heading.
        shooterServoSubsystem.addTelemetry(telemetry);

        telemetry.addLine("--- INTAKE ---");
        telemetry.addData("Intake Control", intakeControl);
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
