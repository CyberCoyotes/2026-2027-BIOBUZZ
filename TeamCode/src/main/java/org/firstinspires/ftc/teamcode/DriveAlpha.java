package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/*
 * Drive Alpha
 * An iterative OpMode that drives the robot with the MecanumDriveSubsystem using gamepad1.
 * It starts ROBOT-CENTRIC: "forward" means the way the robot is facing.
 *
 * DRIVING
 *   Left Stick  -> drive forward / backward and slide (strafe) left / right
 *   Right Stick -> turn (push right to turn right)
 *   HOLD the Right Trigger -> SLOW MODE for careful lining up
 *
 * WHEEL TEST (for checking motor directions, with the robot ON BLOCKS so the wheels are in the air)
 *   HOLD a button to spin ONE wheel at WHEEL_TEST_POWER. It must push the robot FORWARD.
 *     X (Square)   -> left front
 *     A (Cross)    -> left rear
 *     Y (Triangle) -> right front
 *     B (Circle)   -> right rear
 *   The wheel test beats the sticks. If a wheel spins backward, flip THAT motor's direction in
 *   MecanumDriveSubsystem.
 *
 * SAFE FIRST TEST (do these in order):
 *   1. Robot on blocks, wheels in the air. Do the WHEEL TEST above for all four wheels.
 *   2. Robot on the floor with lots of space. Push the left stick FORWARD a little.
 *      The robot must go forward in a straight line.
 *   3. Push the left stick LEFT. The robot must slide left without turning.
 *   4. Push the right stick LEFT. The robot must spin counterclockwise (viewed from above).
 *   5. Try moves together (forward while turning). Then try slow mode.
 *
 * WHY THE STICK SIGNS ARE FLIPPED
 *   The subsystem uses the same convention as Pedro Pathing, Pinpoint and the turret:
 *   forward +, strafe LEFT +, turn COUNTERCLOCKWISE +. The controller's sticks are different:
 *   pushing a stick forward gives a NEGATIVE number, and pushing it right gives a POSITIVE
 *   number. So we flip the sign of all three.
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station):
 *   Motor names: left_front_motor, left_rear_motor, right_front_motor, right_rear_motor
 */
@TeleOp(name = "Drive Alpha", group = "Testing")
public class DriveAlpha extends OpMode {

    // The sticks do not always rest at exactly zero. Anything smaller than this is ignored,
    // so the robot does not creep when you let go.
    private static final double STICK_DEADBAND = 0.05;

    // Holding the right trigger switches to this fraction of full speed (0.4 = 40%).
    private static final double SLOW_MODE_POWER_SCALE = 0.4;

    // The trigger gives a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // Power for the one-wheel direction test. Keep it low and safe.
    private static final double WHEEL_TEST_POWER = 0.4;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDriveSubsystem mecanumDriveSubsystem;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Left Stick = drive and strafe   Right Stick = turn");
        telemetry.addLine("Right Trigger = SLOW MODE");
        telemetry.addLine("WHEEL TEST (robot on blocks!):");
        telemetry.addLine("  X = left front   A = left rear   Y = right front   B = right rear");
    }

    /*
     * Ignores a stick value that is too small to be on purpose.
     */
    private double applyDeadband(double stickValue) {
        if (Math.abs(stickValue) < STICK_DEADBAND) {
            return 0.0;
        }
        return stickValue;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The subsystem gets its own motors from the hardware map.
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap);

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
        // 1. Read sensors. (The drive has none.)

        // 2. Read the gamepad and decide what to do.
        // On the controller, pushing a stick FORWARD gives a negative number, and pushing it RIGHT
        // gives a positive number. Our subsystem wants forward +, LEFT +, and COUNTERCLOCKWISE +,
        // so we flip the sign of all three. (Turning right is clockwise, which is negative.)
        double forward = applyDeadband(-gamepad1.left_stick_y);
        double strafe = applyDeadband(-gamepad1.left_stick_x);
        double turn = applyDeadband(-gamepad1.right_stick_x);

        boolean slowMode = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean testingWheels = gamepad1.x || gamepad1.a || gamepad1.y || gamepad1.b;

        // 3. Tell the drivetrain what to do. (There is no update() to call.)
        String driveControl;
        if (testingWheels) {
            // Spin only the wheels whose buttons are held. The other wheels get 0.
            double leftFrontPower = 0.0;
            double leftRearPower = 0.0;
            double rightFrontPower = 0.0;
            double rightRearPower = 0.0;
            if (gamepad1.x) {
                leftFrontPower = WHEEL_TEST_POWER;
            }
            if (gamepad1.a) {
                leftRearPower = WHEEL_TEST_POWER;
            }
            if (gamepad1.y) {
                rightFrontPower = WHEEL_TEST_POWER;
            }
            if (gamepad1.b) {
                rightRearPower = WHEEL_TEST_POWER;
            }
            mecanumDriveSubsystem.setWheelPowers(leftFrontPower, rightFrontPower,
                    leftRearPower, rightRearPower);
            driveControl = "WHEEL TEST";
        } else {
            if (slowMode) {
                mecanumDriveSubsystem.setPowerScale(SLOW_MODE_POWER_SCALE);
                driveControl = "Sticks (SLOW MODE)";
            } else {
                mecanumDriveSubsystem.setPowerScale(1.0);
                driveControl = "Sticks";
            }
            mecanumDriveSubsystem.drive(forward, strafe, turn);
        }

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Control", driveControl);
        mecanumDriveSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        mecanumDriveSubsystem.stop();
    }
}
