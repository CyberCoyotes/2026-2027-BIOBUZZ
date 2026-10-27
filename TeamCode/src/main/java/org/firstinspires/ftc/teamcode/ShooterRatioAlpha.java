package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/*
 * Shooter Ratio Testing
 * An iterative OpMode for finding the best HOOD WHEEL RATIO, using speed control (RPM).
 * It is for the hood wheel ratio test. The printed sheet is in
 * TeamCode/docs/hood-wheel-ratio-test-sheet.html (open it in a browser and print it).
 *
 * THE IDEA
 *   RATIO = HOOD WHEEL RPM / FLYWHEEL RPM. The hood wheel is smaller (about 2 inches) than the
 *   flywheel (about 4 inches), so a ratio near 2.0 should make the two wheel SURFACES move at
 *   the same speed and give the ball little or no spin. This test finds out what ratio really
 *   gives the least spin, and whether that ratio stays the same at different flywheel speeds.
 *   When you find the best ratio, type it into HOOD_WHEEL_RATIO in ShooterSubsystem.
 *
 * THE TWO DIALS (you turn them with buttons, so you do not have to change code and redeploy)
 *   B           -> flywheel target UP   by FLYWHEEL_RPM_STEP
 *   A           -> flywheel target DOWN by FLYWHEEL_RPM_STEP
 *   D-pad Right -> ratio UP   by 0.05
 *   D-pad Left  -> ratio DOWN by 0.05
 *   The hood wheel target is always flywheel target x ratio. The screen shows all three.
 *   If the hood wheel target is higher than the shooter allows, the screen says CAPPED. Do not
 *   record that shot: the real ratio is lower than the dial says.
 *
 * GAMEPAD 1 (shooter and gate)
 *   Y            -> shooter ON (spins to the dialed speeds and keeps spinning)
 *   X            -> shooter OFF (the wheels coast down. Wait for them to STOP before touching.)
 *   Right Bumper -> HOLD to reverse the flywheel only (clearing a jam)
 *   Left Bumper  -> HOLD to reverse the hood wheel only (clearing a jam)
 *   Left Trigger -> TOGGLES the gate between LOADING and SHOOTING (one flip per press)
 *   D-pad Up/Down-> nudge the gate position (the gate positions are still being measured)
 *   Back (Share) -> reset the "lowest RPM" readings
 *
 * THE "LOWEST RPM" READINGS
 *   When the ball squeezes between the wheels, each wheel slows down for a moment, then the
 *   speed control catches up. The screen remembers the LOWEST RPM each wheel hit since the
 *   shooter first reached speed. Read it after your shots, write it on the sheet, then press
 *   Back to reset. Changing a dial, or turning the shooter off, also resets it.
 *
 * GAMEPAD 2 (intake)
 *   Right Bumper -> INTAKE (pull balls in)     Left Bumper -> REVERSE
 *   The intake runs at the power set in IntakeSubsystem (DEFAULT_INTAKE_POWER).
 *
 * SAFE USE: keep hands, hair and loose clothing away from the wheels, wear eye protection, and
 * keep everyone out of the path of the ball. Only shoot when the screen says "At Speed? true".
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle, Back = Share.
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
    // Where the flywheel dial starts, how far one press moves it, and how low and high it can go.
    // All are in RPM.
    // The flywheel motor is the goBILDA 1620 RPM one now, so the dial stops at 1450, just under
    // the shooter's own limit (FLYWHEEL_MAX_TARGET_RPM, 1458 in ShooterSubsystem).
    private static final int FLYWHEEL_RPM_START = 1300;
    private static final int FLYWHEEL_RPM_STEP = 100;
    private static final int FLYWHEEL_RPM_MIN = 500;
    private static final int FLYWHEEL_RPM_MAX = 1450;

    // The ratio dial is counted in HUNDREDTHS so that adding 5 over and over never drifts to
    // something like 1.9999999. So 200 means a ratio of 2.00, and a step of 5 means 0.05.
    private static final int RATIO_HUNDREDTHS_START = 200;
    private static final int RATIO_HUNDREDTHS_STEP = 5;
    private static final int RATIO_HUNDREDTHS_MIN = 100;
    private static final int RATIO_HUNDREDTHS_MAX = 300;

    // How many hundredths make a whole number (200 hundredths = 2.00).
    private static final double HUNDREDTHS_PER_ONE = 100.0;

    // ------------------------------------------------------------------
    // Other test numbers
    // ------------------------------------------------------------------
    // Reverse power is NEGATIVE so the motor spins the other way.
    private static final double FLYWHEEL_POWER_REVERSE = -0.60;
    private static final double HOOD_WHEEL_POWER_REVERSE = -0.60;

    // The triggers give a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // How far one D-pad press moves the gate servo position (the position runs 0.0 to 1.0).
    // 0.01 is about 3 degrees on a 300 degree servo (0.01 x 300).
    private static final double GATE_NUDGE_STEP = 0.01;

    // The measured ratio (hood RPM / flywheel RPM) is only worth showing when the flywheel is
    // spinning faster than this. Below it the division gives silly numbers.
    private static final double MIN_RPM_FOR_MEASURED_RATIO = 300.0;

    // A hood wheel target counts as CAPPED when the shooter's real target is lower than the
    // dialed one by more than this many RPM.
    private static final double CAPPED_TOLERANCE_RPM = 1.0;

    // ------------------------------------------------------------------
    // true = the control map stays on the Driver Station screen while the OpMode RUNS (at the
    // bottom, under the live data). false = it is only shown before START, like it used to be.
    private static final boolean SHOW_CONTROLS_WHILE_RUNNING = true;

    // Declare OpMode members.
    // ------------------------------------------------------------------
    private ElapsedTime runtime = new ElapsedTime();
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private IntakeSubsystem intakeSubsystem;

    // The dials, in whole numbers.
    private int flywheelRpmDial = FLYWHEEL_RPM_START;
    private int ratioHundredths = RATIO_HUNDREDTHS_START;

    // True after Y, false after X. The shooter keeps spinning between button presses.
    private boolean shooterOn = false;

    // The lowest RPM each wheel hit since the shooter first reached speed.
    // We do not start watching until the wheels are at speed the first time, because the
    // spin-up from zero would otherwise count as a "low".
    private boolean watchingLowest = false;
    private double lowestFlywheelRpm = 0.0;
    private double lowestHoodWheelRpm = 0.0;

    // What the buttons were doing LAST loop. A press lasts many loops (about 50 per second),
    // but we want it to count only ONCE. So we only act when it was NOT pressed last loop
    // and IS pressed now.
    private boolean wasYPressed = false;
    private boolean wasXPressed = false;
    private boolean wasBPressed = false;
    private boolean wasAPressed = false;
    private boolean wasDpadLeftPressed = false;
    private boolean wasDpadRightPressed = false;
    private boolean wasBackPressed = false;
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
     * Forget the "lowest RPM" readings and start watching again from the next time the
     * wheels reach speed.
     */
    private void resetLowestReadings() {
        watchingLowest = false;
    }

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- GAMEPAD 1 ---");
        telemetry.addLine("Y = shooter ON   X = shooter OFF");
        telemetry.addLine("B / A = flywheel RPM up / down");
        telemetry.addLine("D-pad Right / Left = ratio up / down");
        telemetry.addLine("Back (Share) = reset the LOWEST RPM");
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

        // Show the gate here too, so you can check before START that it went to LOADING.
        shooterServoSubsystem.addTelemetry(telemetry);
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

        // 2. Read the gamepads and decide what to do.
        boolean yPressed = gamepad1.y;
        boolean xPressed = gamepad1.x;
        boolean bPressed = gamepad1.b;
        boolean aPressed = gamepad1.a;
        boolean dpadLeftPressed = gamepad1.dpad_left;
        boolean dpadRightPressed = gamepad1.dpad_right;
        boolean backPressed = gamepad1.back;
        boolean gateTogglePressed = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean gateNudgeUpPressed = gamepad1.dpad_up;
        boolean gateNudgeDownPressed = gamepad1.dpad_down;

        // Shooter on and off. If both are pressed at once, OFF wins (the safer choice).
        if (isNewPress(yPressed, wasYPressed)) {
            shooterOn = true;
            resetLowestReadings();
        }
        if (isNewPress(xPressed, wasXPressed)) {
            shooterOn = false;
            resetLowestReadings();
        }

        // Turn the dials. Any change makes the old "lowest RPM" readings meaningless.
        if (isNewPress(bPressed, wasBPressed)) {
            flywheelRpmDial = flywheelRpmDial + FLYWHEEL_RPM_STEP;
            resetLowestReadings();
        }
        if (isNewPress(aPressed, wasAPressed)) {
            flywheelRpmDial = flywheelRpmDial - FLYWHEEL_RPM_STEP;
            resetLowestReadings();
        }
        if (isNewPress(dpadRightPressed, wasDpadRightPressed)) {
            ratioHundredths = ratioHundredths + RATIO_HUNDREDTHS_STEP;
            resetLowestReadings();
        }
        if (isNewPress(dpadLeftPressed, wasDpadLeftPressed)) {
            ratioHundredths = ratioHundredths - RATIO_HUNDREDTHS_STEP;
            resetLowestReadings();
        }
        if (isNewPress(backPressed, wasBackPressed)) {
            resetLowestReadings();
        }

        // Keep the dials between their lowest and highest settings.
        if (flywheelRpmDial > FLYWHEEL_RPM_MAX) {
            flywheelRpmDial = FLYWHEEL_RPM_MAX;
        }
        if (flywheelRpmDial < FLYWHEEL_RPM_MIN) {
            flywheelRpmDial = FLYWHEEL_RPM_MIN;
        }
        if (ratioHundredths > RATIO_HUNDREDTHS_MAX) {
            ratioHundredths = RATIO_HUNDREDTHS_MAX;
        }
        if (ratioHundredths < RATIO_HUNDREDTHS_MIN) {
            ratioHundredths = RATIO_HUNDREDTHS_MIN;
        }

        // The hood wheel target is the flywheel target times the ratio.
        double ratio = ratioHundredths / HUNDREDTHS_PER_ONE;
        double flywheelTargetRpm = flywheelRpmDial;
        double hoodWheelTargetRpm = flywheelTargetRpm * ratio;

        // 3. Tell the subsystems what to do, then call their update().

        // Shooter wheels. Reverse beats everything, so clearing a jam always works.
        String shooterControl;
        boolean reverseFlywheel = gamepad1.right_bumper;
        boolean reverseHoodWheel = gamepad1.left_bumper;
        if (reverseFlywheel || reverseHoodWheel) {
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
            resetLowestReadings();
        } else if (shooterOn) {
            shooterSubsystem.setTargetRpms(flywheelTargetRpm, hoodWheelTargetRpm);
            shooterControl = "ON";
        } else {
            shooterSubsystem.stop();
            shooterControl = "OFF";
        }
        shooterSubsystem.update();

        // Remember the lowest RPM each wheel hit. Start watching only once the wheels have
        // reached speed, so the spin-up from zero does not count.
        if (shooterOn && shooterSubsystem.isAtSpeed()) {
            if (!watchingLowest) {
                watchingLowest = true;
                lowestFlywheelRpm = shooterSubsystem.getFlywheelRpm();
                lowestHoodWheelRpm = shooterSubsystem.getHoodWheelRpm();
            }
        }
        if (watchingLowest) {
            lowestFlywheelRpm = Math.min(lowestFlywheelRpm, shooterSubsystem.getFlywheelRpm());
            lowestHoodWheelRpm = Math.min(lowestHoodWheelRpm, shooterSubsystem.getHoodWheelRpm());
        }

        // Gate.
        if (isNewPress(gateTogglePressed, wasGateTogglePressed)) {
            shooterServoSubsystem.toggle();
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
        wasBackPressed = backPressed;
        wasGateTogglePressed = gateTogglePressed;
        wasGateNudgeUpPressed = gateNudgeUpPressed;
        wasGateNudgeDownPressed = gateNudgeDownPressed;

        // 4. Show what is happening on the Driver Station, one section at a time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addLine("--- THE DIALS ---");
        telemetry.addData("Flywheel target RPM", "%d", flywheelRpmDial);
        telemetry.addData("Ratio (hood / flywheel)", "%.2f", ratio);

        // The shooter cuts any target above its maximum down to the maximum. Warn if that
        // happened to the hood wheel, because then the real ratio is lower than the dial says.
        boolean hoodWheelCapped = hoodWheelTargetRpm
                > shooterSubsystem.getMaxHoodWheelTargetRpm() + CAPPED_TOLERANCE_RPM;
        if (hoodWheelCapped) {
            telemetry.addData("Hood wheel target RPM", "%.0f  CAPPED at %.0f - DO NOT RECORD",
                    hoodWheelTargetRpm, shooterSubsystem.getMaxHoodWheelTargetRpm());
        } else {
            telemetry.addData("Hood wheel target RPM", "%.0f", hoodWheelTargetRpm);
        }

        telemetry.addLine("--- SHOOTER ---");
        telemetry.addData("Shooter switch", shooterControl);
        shooterSubsystem.addTelemetry(telemetry);
        if (shooterSubsystem.getFlywheelRpm() > MIN_RPM_FOR_MEASURED_RATIO) {
            telemetry.addData("Ratio measured now", "%.2f",
                    shooterSubsystem.getHoodWheelRpm() / shooterSubsystem.getFlywheelRpm());
        }

        telemetry.addLine("--- LOWEST RPM SINCE REACHING SPEED (Back resets) ---");
        if (watchingLowest) {
            telemetry.addData("Flywheel lowest", "%.0f RPM", lowestFlywheelRpm);
            telemetry.addData("Hood wheel lowest", "%.0f RPM", lowestHoodWheelRpm);
        } else {
            telemetry.addData("Lowest", "waiting for the wheels to reach speed");
        }

        // The gate prints its own "--- SHOOTER GATE ---" heading.
        shooterServoSubsystem.addTelemetry(telemetry);

        telemetry.addLine("--- INTAKE ---");
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
