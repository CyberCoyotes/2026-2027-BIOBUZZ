package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Mecanum Drive Subsystem
 * Owns the four drive motors. Only this class touches them in TeleOp.
 * OpModes tell it where to go with drive(forward, strafe, turn).
 *
 * WHEN TO USE IT
 *   This subsystem is for TELEOP. In AUTO, Pedro Pathing's Follower drives these same four
 *   motors, and only ONE class may control a motor. Never use both in the same OpMode.
 *   (Pedro's motor names and directions must match the ones in this file.)
 *
 * THE CONVENTION (the same one Pedro Pathing, Pinpoint and our turret use)
 *   Looking down from above, with the robot's front at the top:
 *
 *                 +forward
 *                    ^
 *      +strafe  <--  O  -->  -strafe        (strafe LEFT is POSITIVE)
 *      (left)        |        (right)
 *
 *   +turn spins the robot COUNTERCLOCKWISE (toward the left). -turn spins it clockwise.
 *   Every input is a number from -1.0 to 1.0.
 *   The controller's sticks use the opposite signs, so the OpMode flips them (see DriveAlpha).
 *
 * HOW THE WHEELS MAKE THE MOVES (mecanum wheels, rollers in an X when viewed from above)
 *   leftFront  = forward - strafe - turn
 *   rightFront = forward + strafe + turn
 *   leftRear   = forward + strafe - turn
 *   rightRear  = forward - strafe + turn
 *   If any wheel would be asked for more than 100%, all four wheels are scaled down together,
 *   so the robot still moves in the direction the driver asked for, just a little slower.
 *
 * STATE (what we told it to do) is the last drive() inputs, the power scale, and the wheel powers.
 * There are no sensors here, so there is no STATUS and no update() to call.
 *
 * This first version is ROBOT-CENTRIC: "forward" means the way the robot is facing.
 *
 * Robot configuration (on the Driver Station) for ALL three robots. The names MUST match:
 *   Motor name: left_front_motor
 *   Motor name: left_rear_motor
 *   Motor name: right_front_motor
 *   Motor name: right_rear_motor
 */
public class MecanumDriveSubsystem {

    // ------------------------------------------------------------------
    // Hardware names. These MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    private static final String LEFT_FRONT_MOTOR_NAME = "left_front";
    private static final String LEFT_REAR_MOTOR_NAME = "left_rear";
    private static final String RIGHT_FRONT_MOTOR_NAME = "right_front";
    private static final String RIGHT_REAR_MOTOR_NAME = "right_rear";

    // ------------------------------------------------------------------
    // Directions. TEST THESE on the robot with the wheels off the floor!
    // ------------------------------------------------------------------
    // The motors on the two sides face opposite ways, so one side usually needs REVERSE.
    // These start the same as the FTC sample code, which assumes the wheels turn the same way
    // as the motor shaft.
    // Test: each wheel must push the robot FORWARD when it is given positive power.
    // If a wheel spins backward, flip THAT motor's line between FORWARD and REVERSE.
    private static final DcMotor.Direction LEFT_FRONT_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction LEFT_REAR_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_FRONT_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction RIGHT_REAR_DIRECTION = DcMotor.Direction.FORWARD;

    // ------------------------------------------------------------------
    // Driving settings
    // ------------------------------------------------------------------
    // BRAKE makes the robot stop quickly and lets it hold its spot, which helps when parking.
    // But the robot is TALL (up to 29 inches, with a turret and shooter on top), so stopping
    // very suddenly from full speed could tip it. If that happens, try FLOAT, or ask for a
    // limit on how fast the power can change.
    private static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.BRAKE;

    // The most power any wheel will ever get (0.0 to 1.0). 1.0 is full speed.
    // Lower this for the first tests, or if the robot is too fast for new drivers.
    private static final double MAX_DRIVE_POWER = 1.0;

    // ------------------------------------------------------------------
    // Hardware and data. Only this class changes these.
    // ------------------------------------------------------------------
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;

    // 1.0 is full speed. Slow mode lowers it (see setPowerScale).
    private double powerScale = 1.0;

    // What we last asked for, kept so telemetry can show it.
    private double forwardInput = 0.0;
    private double strafeInput = 0.0;
    private double turnInput = 0.0;
    private double leftFrontPower = 0.0;
    private double rightFrontPower = 0.0;
    private double leftRearPower = 0.0;
    private double rightRearPower = 0.0;

    /*
     * Setup. Gets the four motors from the robot configuration and sets them up.
     * The robot starts stopped.
     */
    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        // The names MUST match the robot configuration exactly.
        leftFrontMotor = hardwareMap.get(DcMotor.class, LEFT_FRONT_MOTOR_NAME);
        leftRearMotor = hardwareMap.get(DcMotor.class, LEFT_REAR_MOTOR_NAME);
        rightFrontMotor = hardwareMap.get(DcMotor.class, RIGHT_FRONT_MOTOR_NAME);
        rightRearMotor = hardwareMap.get(DcMotor.class, RIGHT_REAR_MOTOR_NAME);

        setUpHardware();
    }

    /*
     * Sets directions and modes, then makes sure every wheel is stopped.
     */
    private void setUpHardware() {
        leftFrontMotor.setDirection(LEFT_FRONT_DIRECTION);
        leftRearMotor.setDirection(LEFT_REAR_DIRECTION);
        rightFrontMotor.setDirection(RIGHT_FRONT_DIRECTION);
        rightRearMotor.setDirection(RIGHT_REAR_DIRECTION);

        // We control the wheels with raw power. Pinpoint keeps track of where the robot is,
        // so we do not need the drive motors' encoders for anything here.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        leftRearMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        rightFrontMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        rightRearMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

        setWheelPowers(0.0, 0.0, 0.0, 0.0);
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the drivetrain what to do
    // ==================================================================

    /*
     * Drive the robot. Each number is from -1.0 to 1.0:
     *   forward: + drives forward,        - drives backward
     *   strafe:  + slides LEFT,           - slides right
     *   turn:    + spins COUNTERCLOCKWISE (left), - spins clockwise (right)
     * Call this every loop. Send all zeros (or call stop()) to stop.
     *
     * The wheel powers are worked out first, then scaled down together if any wheel would be
     * over 100%, and then slowed by the power scale (slow mode) and MAX_DRIVE_POWER.
     */
    public void drive(double forward, double strafe, double turn) {
        forwardInput = Range.clip(forward, -1.0, 1.0);
        strafeInput = Range.clip(strafe, -1.0, 1.0);
        turnInput = Range.clip(turn, -1.0, 1.0);

        // Combine the three moves into a power for each wheel.
        double leftFront = forwardInput - strafeInput - turnInput;
        double rightFront = forwardInput + strafeInput + turnInput;
        double leftRear = forwardInput + strafeInput - turnInput;
        double rightRear = forwardInput - strafeInput + turnInput;

        // Find the biggest power any wheel was asked for.
        double largestPower = Math.abs(leftFront);
        largestPower = Math.max(largestPower, Math.abs(rightFront));
        largestPower = Math.max(largestPower, Math.abs(leftRear));
        largestPower = Math.max(largestPower, Math.abs(rightRear));

        // If a wheel was asked for more than 100%, shrink ALL four by the same amount.
        // That keeps the wheels in the same proportion, so the robot still goes where we asked.
        if (largestPower > 1.0) {
            leftFront = leftFront / largestPower;
            rightFront = rightFront / largestPower;
            leftRear = leftRear / largestPower;
            rightRear = rightRear / largestPower;
        }

        // Apply slow mode and the top speed limit.
        double scale = powerScale * MAX_DRIVE_POWER;
        setWheelPowers(leftFront * scale, rightFront * scale, leftRear * scale, rightRear * scale);
    }

    /*
     * Slow mode. 1.0 is full speed, 0.4 is 40% speed. Use a low number for careful lining up.
     * It stays set until you change it, so call it every loop (or only when the driver
     * changes it).
     */
    public void setPowerScale(double scale) {
        powerScale = Range.clip(scale, 0.0, 1.0);
    }

    /*
     * Send a RAW power (-1.0 to 1.0) to each wheel. This skips the mecanum math, the
     * normalizing, and slow mode. drive() uses it to send its answer to the motors, and the
     * wheel test in DriveAlpha uses it to spin one wheel at a time.
     * Positive power should push the robot FORWARD (see the direction notes at the top).
     */
    public void setWheelPowers(double leftFront, double rightFront,
                               double leftRear, double rightRear) {
        leftFrontPower = Range.clip(leftFront, -1.0, 1.0);
        rightFrontPower = Range.clip(rightFront, -1.0, 1.0);
        leftRearPower = Range.clip(leftRear, -1.0, 1.0);
        rightRearPower = Range.clip(rightRear, -1.0, 1.0);

        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightRearMotor.setPower(rightRearPower);
    }

    /*
     * Stop all four wheels. Safe to call every loop.
     */
    public void stop() {
        forwardInput = 0.0;
        strafeInput = 0.0;
        turnInput = 0.0;
        setWheelPowers(0.0, 0.0, 0.0, 0.0);
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks the drivetrain what is happening
    // ==================================================================

    /*
     * Adds the drivetrain's own data to the telemetry. The SDK sends telemetry to the
     * Driver Station after every loop(), so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive inputs", "forward %.2f   strafe %.2f   turn %.2f",
                forwardInput, strafeInput, turnInput);
        telemetry.addData("Power scale", "%.0f%%", powerScale * 100);
        telemetry.addData("Front left/right", "%.2f, %.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Rear  left/right", "%.2f, %.2f", leftRearPower, rightRearPower);
    }
}
