package org.firstinspires.ftc.teamcode;

/*
 * Numbers that BiobuzzTeleOp and the Alpha test OpModes both use.
 *
 * WHY THIS FILE EXISTS
 *   BiobuzzTeleOp is DriveAlpha, IntakeAlpha, ShooterRatioAlpha and TurretAlpha put together,
 *   so it used the same numbers as they do. Before this file, each OpMode kept its own copy,
 *   and changing a number meant remembering to change it in every copy.
 *   Now each of these numbers lives HERE, once. Change it here and every OpMode sees it.
 *
 * WHAT DOES NOT GO HERE
 *   - Numbers that describe a mechanism (motor directions, servo positions, the best intake
 *     power, RPM limits) live in that mechanism's subsystem class.
 *   - Numbers only ONE OpMode uses (like the Y / X preset powers in ShooterPowerAlpha) stay at
 *     the top of that OpMode.
 *   - Add a number to this file only when a SECOND OpMode needs it.
 *
 * HOW TO USE A NUMBER
 *   Write the class name in front of it: OpModeConstants.TRIGGER_PRESSED_THRESHOLD
 *
 * This is a plain class that holds numbers. It has no hardware and no methods.
 * It is NOT called "Constants" because Pedro Pathing's setup has its own Constants class.
 */
public class OpModeConstants {

    // ------------------------------------------------------------------
    // Gamepad feel
    // ------------------------------------------------------------------
    // The triggers give a number from 0.0 to 1.0. Past this much counts as "pressed".
    public static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // The driving sticks do not always rest at exactly zero. Anything smaller than this is
    // ignored, so the robot does not creep when you let go.
    public static final double DRIVE_STICK_DEADBAND = 0.05;

    // The turret stick must move more than this before it counts, so a stick that does not
    // quite rest at zero will not turn the turret.
    public static final double TURRET_STICK_DEADBAND = 0.1;

    // ------------------------------------------------------------------
    // Shooter: reverse powers (for clearing a jam)
    // ------------------------------------------------------------------
    // Reverse power is NEGATIVE so the motor spins the other way.
    public static final double FLYWHEEL_POWER_REVERSE = -0.60;
    public static final double HOOD_WHEEL_POWER_REVERSE = -0.60;

    // ------------------------------------------------------------------
    // Shooter gate
    // ------------------------------------------------------------------
    // How far one D-pad press moves the gate servo position (the position runs 0.0 to 1.0).
    // 0.01 is about 3 degrees on a 300 degree servo (0.01 x 300). Make it smaller for
    // finer adjustment, or bigger to move faster.
    public static final double GATE_NUDGE_STEP = 0.01;

    // ------------------------------------------------------------------
    // Shooter: the two dials from the ratio test (these are test-only numbers)
    // ------------------------------------------------------------------
    // The flywheel power dial is counted in WHOLE PERCENT (30 means 30% power).
    // Where it starts, how far one press moves it, and how low and high it can go.
    public static final int FLYWHEEL_POWER_PERCENT_START = 30;
    public static final int FLYWHEEL_POWER_PERCENT_STEP = 5;
    public static final int FLYWHEEL_POWER_PERCENT_MIN = 5;
    public static final int FLYWHEEL_POWER_PERCENT_MAX = 100;

    // The ratio dial is counted in HUNDREDTHS so that adding 5 over and over never drifts to
    // something like 1.9999999. So 200 means a ratio of 2.00, and a step of 5 means 0.05.
    public static final int RATIO_HUNDREDTHS_START = 200;
    public static final int RATIO_HUNDREDTHS_STEP = 5;
    public static final int RATIO_HUNDREDTHS_MIN = 100;
    public static final int RATIO_HUNDREDTHS_MAX = 300;

    // How many hundredths make a whole number (200 hundredths = 2.00).
    public static final double HUNDREDTHS_PER_ONE = 100.0;

    // How many percent make a whole power of 1.0 (100 percent = 1.0 power).
    public static final double PERCENT_PER_ONE = 100.0;

    // The most power a wheel can be given, in percent. A hood wheel power above this is CAPPED.
    public static final double MAX_POWER_PERCENT = 100.0;

    // How long the wheels need to settle after the shooter starts, a dial changes, or a ball
    // goes through. A starting guess. Time it with a stopwatch: how long until the wheels
    // sound steady?
    public static final double SPIN_SETTLE_SECONDS = 3.0;

    // ------------------------------------------------------------------
    // Intake power dial (test-only)
    // ------------------------------------------------------------------
    // How much one D-pad press changes the dial, in percent. 5 means 5%.
    public static final int INTAKE_DIAL_STEP_PERCENT = 5;

    // The lowest and highest the dial can go, in percent.
    public static final int INTAKE_DIAL_MIN_PERCENT = 0;
    public static final int INTAKE_DIAL_MAX_PERCENT = 100;

    // ------------------------------------------------------------------
    // Turret power dial and preset angles (test-only)
    // ------------------------------------------------------------------
    // The turret power at the start, in percent. A full push on the stick, or holding D-pad
    // Left / Right, turns the turret at this power. D-pad Up / Down change it. 100 is full power.
    public static final int TURRET_POWER_PERCENT_START = 50;

    // Each press of D-pad Up or Down changes the turret power by this many percent.
    public static final int TURRET_POWER_PERCENT_STEP = 5;

    // The turret power stays between these two numbers.
    public static final int TURRET_POWER_PERCENT_MIN = 0;
    public static final int TURRET_POWER_PERCENT_MAX = 100;

    // The turret preset angles. LEFT is POSITIVE and RIGHT is NEGATIVE, the same as Pinpoint
    // and Pedro Pathing. Center is 0 degrees, which is 0 encoder ticks.
    public static final double TURRET_PRESET_SIDE_DEGREES = 90.0;
    public static final double TURRET_PRESET_LEFT_DEGREES = TURRET_PRESET_SIDE_DEGREES;
    public static final double TURRET_PRESET_RIGHT_DEGREES = -TURRET_PRESET_SIDE_DEGREES;
    public static final double TURRET_PRESET_CENTER_DEGREES = 0.0;
}
