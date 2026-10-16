package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/*
 * TeleOp Alpha 3
 * An iterative OpMode that DRIVES the robot and runs the INTAKE, both on gamepad1.
 * It is DriveAlpha (driving) and IntakeAlpha (intake) put together, so each part has
 * already been tested on its own. This OpMode owns NO hardware. It only reads the gamepad
 * and tells the subsystems what to do. (Shooter and turret will be added in later alphas.)
 *
 * It starts ROBOT-CENTRIC: "forward" means the way the robot is facing.
 *
 * DRIVING
 *   Left Stick  -> drive forward / backward and slide (strafe) left / right
 *   Right Stick -> turn (push right to turn right)
 *   HOLD the Left Trigger -> SLOW MODE for careful lining up
 *
 * INTAKE (HOLD a bumper to run)
 *   Right Bumper -> INTAKE (pull balls in)
 *   Left Bumper  -> REVERSE (push balls out)
 *   No bumper    -> intake stops
 *   Left Bumper beats Right Bumper, so clearing a jam always works.
 *   The intake runs at the power set in IntakeSubsystem (DEFAULT_INTAKE_POWER).
 *   To change it, change that number there. (IntakeAlpha has the power dial for testing.)
 *
 * THE INTAKE ALSO RUNS THE TRANSFER. One motor spins the front intake AND a hex shaft with
 * grippers in the transfer chute, so balls are carried up the chute while you drive.
 *
 * NOT IN THIS OPMODE: the one-wheel direction test. Use DriveAlpha for that, with the
 * robot on blocks.
 *
 * WHY THE STICK SIGNS ARE FLIPPED
 *   The drive subsystem uses the same convention as Pedro Pathing, Pinpoint and the turret:
 *   forward +, strafe LEFT +, turn COUNTERCLOCKWISE +. The controller's sticks are different:
 *   pushing a stick forward gives a NEGATIVE number, and pushing it right gives a POSITIVE
 *   number. So we flip the sign of all three.
 *
 * On a PS5 controller: the bumpers are L1 and R1, the triggers are L2 and R2.
 *
 * Robot configuration (on the Driver Station). The names live in the subsystem classes:
 *   Motor names: left_front, left_rear, right_front, right_rear   (MecanumDriveSubsystem)
 *   Motor name:  intake                                           (IntakeSubsystem)
 */
@TeleOp(name = "TeleOp - Alpha 3", group = "Testing")
public class TeleOpAlpha extends OpMode {

    // The sticks do not always rest at exactly zero. Anything smaller than this is ignored,
    // so the robot does not creep when you let go.
    private static final double STICK_DEADBAND = 0.05;

    // Holding the left trigger switches to this fraction of full speed (0.4 = 40%).
    private static final double SLOW_MODE_POWER_SCALE = 0.4;

    // The speed scale when slow mode is NOT held. 1.0 is full speed.
    private static final double NORMAL_POWER_SCALE = 1.0;

    // The trigger gives a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private IntakeSubsystem intakeSubsystem;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Left Stick = drive and strafe   Right Stick = turn");
        telemetry.addLine("Left Trigger = SLOW MODE");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
    }

    /*
     * Shows the RAW numbers from gamepad1, before any deadband or sign flip.
     * This is for finding a controller problem: with your hands OFF the controller, every stick
     * should read about 0.00, both triggers 0.00, and every button false. Anything else means
     * the controller (or how the Driver Station sees it) is the problem, not the drive code.
     * It is only shown before START, so it does not crowd the screen while driving.
     */
    private void addRawGamepadTelemetry() {
        telemetry.addLine("--- RAW GAMEPAD 1 (hands off = all zeros / false) ---");
        telemetry.addData("Controller type", gamepad1.type());
        telemetry.addData("Left stick x, y", "%.2f, %.2f",
                gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Right stick x, y", "%.2f, %.2f",
                gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("Triggers L, R", "%.2f, %.2f",
                gamepad1.left_trigger, gamepad1.right_trigger);
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
        // Each subsystem gets its own motors from the hardware map.
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap);
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
        addRawGamepadTelemetry();
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
        // 1. Read sensors. (The drive has none. The intake reads its own encoder in update().)
        intakeSubsystem.update();

        // 2. Read the gamepad and decide what to do.
        // On the controller, pushing a stick FORWARD gives a negative number, and pushing it RIGHT
        // gives a positive number. Our subsystem wants forward +, LEFT +, and COUNTERCLOCKWISE +,
        // so we flip the sign of all three. (Turning right is clockwise, which is negative.)
        double forward = applyDeadband(-gamepad1.left_stick_y);
        double strafe = applyDeadband(-gamepad1.left_stick_x);
        double turn = applyDeadband(-gamepad1.right_stick_x);

        boolean slowMode = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;

        // 3. Tell the subsystems what to do. (The drive has no update() to call.)

        // Drive.
        String driveControl;
        if (slowMode) {
            mecanumDriveSubsystem.setPowerScale(SLOW_MODE_POWER_SCALE);
            driveControl = "SLOW MODE";
        } else {
            mecanumDriveSubsystem.setPowerScale(NORMAL_POWER_SCALE);
            driveControl = "Normal";
        }
        mecanumDriveSubsystem.drive(forward, strafe, turn);

        // Intake. Check REVERSE first, so clearing a jam always works.
        String intakeControl;
        if (gamepad1.left_bumper) {
            intakeSubsystem.reverse();
            intakeControl = "Left Bumper (REVERSE)";
        } else if (gamepad1.right_bumper) {
            intakeSubsystem.intake();
            intakeControl = "Right Bumper (INTAKE)";
        } else {
            // No bumper held, so stop the intake.
            intakeSubsystem.stop();
            intakeControl = "None";
        }

        // 4. Show what is happening on the Driver Station, one section at a time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive Control", driveControl);
        telemetry.addData("Drive inputs", "forward %.2f   strafe %.2f   turn %.2f",
                forward, strafe, turn);

        // The intake prints its own "--- INTAKE ---" heading, so we add our line under it.
        intakeSubsystem.addTelemetry(telemetry);
        telemetry.addData("Intake Control", intakeControl);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        mecanumDriveSubsystem.stop();
        intakeSubsystem.stop();
    }
}
