package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;

/*
 * TeleOp Alpha 3
 * An iterative OpMode that DRIVES the robot, runs the INTAKE, and moves the shooter GATE,
 * all on gamepad1. It is DriveAlpha (driving) and IntakeAlpha (intake) put together, plus
 * the gate servo from the shooter tests. This OpMode owns NO hardware. It only reads the
 * gamepad and tells the subsystems what to do. (Shooter wheels and turret will be added in
 * later alphas.)
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
 * SHOOTER GATE
 *   A (Cross on PS5) -> TOGGLES the gate between LOADING and SHOOTING (one flip per press)
 *   The gate starts in LOADING when you hit INIT. The two positions are placeholders until we
 *   measure them. Use ShooterPowerAlpha (D-pad nudge) to find them, then type them into
 *   ShooterServoSubsystem.
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
 *   Servo name:  shooter_servo   (ShooterServoSubsystem, goBILDA dual mode, POSITIONAL mode)
 */
@TeleOp(name = "TeleOp - Alpha 3", group = "Testing")
public class TeleOpAlpha extends OpMode {

    // The sticks do not always rest at exactly zero. Anything smaller than this is ignored,
    // so the robot does not creep when you let go.
    private static final double STICK_DEADBAND = 0.05;

    // The trigger gives a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;

    // Whether button A was pressed LAST loop. A press lasts many loops (about 50 per second),
    // but we want it to flip the gate only ONCE. So we only act when it was NOT pressed last
    // loop and IS pressed now.
    private boolean wasGateButtonPressed = false;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Left Stick = drive and strafe   Right Stick = turn");
        telemetry.addLine("Left Trigger = SLOW MODE");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
        telemetry.addLine("A = TOGGLE shooter gate (loading / shooting)");
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
        // The gate servo moves to LOADING here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);

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
        // How slow "slow mode" is lives in MecanumDriveSubsystem (SLOW_MODE_POWER_SCALE).
        mecanumDriveSubsystem.setSlowMode(slowMode);
        String driveControl;
        if (slowMode) {
            driveControl = "SLOW MODE";
        } else {
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

        // Shooter gate. A counts as ONE press only on the loop where it goes from "not pressed"
        // to "pressed". Without this check, holding A would flip the gate every loop.
        boolean gateButtonPressed = gamepad1.a;
        if (gateButtonPressed && !wasGateButtonPressed) {
            shooterServoSubsystem.toggle();
        }
        wasGateButtonPressed = gateButtonPressed;

        // 4. Show what is happening on the Driver Station, one section at a time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive Control", driveControl);
        telemetry.addData("Drive inputs", "forward %.2f   strafe %.2f   turn %.2f",
                forward, strafe, turn);

        // The intake prints its own "--- INTAKE ---" heading, so we add our line under it.
        intakeSubsystem.addTelemetry(telemetry);
        telemetry.addData("Intake Control", intakeControl);

        // The gate prints its own "--- SHOOTER GATE ---" heading.
        shooterServoSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped.
        // The gate servo just stays where it is. A servo holds its position by itself.
        mecanumDriveSubsystem.stop();
        intakeSubsystem.stop();
    }
}
