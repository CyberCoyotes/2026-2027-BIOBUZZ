package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/*
 * Alpha-Intake Testing
 * An iterative OpMode that runs the intake with the IntakeSubsystem using gamepad1.
 * HOLD a bumper to run the intake. There is ONE power dial, and the D-pad turns it up and down.
 *
 *   Right Bumper -> INTAKE (pull balls in) at the dial power
 *   Left Bumper  -> REVERSE (push balls out) at the dial power
 *   D-pad Up     -> turn the dial UP by 5% (each press)
 *   D-pad Down   -> turn the dial DOWN by 5% (each press)
 *   No bumper    -> intake stops
 *
 * Left Bumper beats Right Bumper, so clearing a jam always works.
 *
 * THE POWER DIAL IS FOR TESTING ONLY
 *   The D-pad is NOT how the real TeleOp will set the intake power. When this test shows us the
 *   best number, change DEFAULT_INTAKE_POWER at the top of IntakeSubsystem.
 *
 * THE INTAKE ALSO RUNS THE TRANSFER
 *   One motor spins the front intake AND a hex shaft with grippers in the transfer chute.
 *   So balls get pulled in and carried up the chute together. Test with a few balls loaded in
 *   the chute, not just the empty robot.
 *
 * SAFE FIRST TEST (do these in order):
 *   1. Robot on blocks. Start the dial at 50%. HOLD the Right Bumper. The intake must pull
 *      IN. If it pushes OUT, flip INTAKE_DIRECTION in IntakeSubsystem.
 *   2. Check the encoder speed on the screen goes UP while the intake runs. (If it always reads 0,
 *      check that the encoder cable is plugged in.)
 *   3. Feed in balls one at a time, then a few together. Use the D-pad to find the lowest power
 *      that works reliably and the power where it starts to jam or throw balls around.
 *   4. HOLD the Left Bumper to check that REVERSE clears a ball.
 *
 * On a PS5 controller: the bumpers are L1 and R1.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: intake_motor
 *   Motor type: goBILDA 5202/3/4 series
 */
@TeleOp(name = "Intake Testing - Alpha", group = "Testing")
public class IntakeAlpha extends OpMode {

    // How much one D-pad press changes the dial, in percent. 5 means 5%.
    private static final int DIAL_STEP_PERCENT = 5;

    // The lowest and highest the dial can go, in percent.
    private static final int DIAL_MIN_PERCENT = 0;
    private static final int DIAL_MAX_PERCENT = 100;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private IntakeSubsystem intakeSubsystem;

    // The dial in WHOLE percent (50 means 50%). We count in whole numbers so adding 5 over and
    // over never drifts to something like 0.4999999.
    private int dialPercent;

    // What the D-pad was doing LAST loop. A D-pad press lasts many loops, but we want it to
    // count only ONCE. So we only act when it was NOT pressed last loop and IS pressed now.
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
        telemetry.addLine("D-pad Up / Down = power dial +5% / -5%");
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The subsystem gets its own motor from the hardware map.
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        // Start the dial wherever the subsystem starts it, so the number only lives in one place.
        // Math.round gives the nearest whole number (0.5 * 100 = 50).
        dialPercent = (int) Math.round(intakeSubsystem.getIntakePower() * 100);

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
        telemetry.addData("Power dial", "%d%%", dialPercent);
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
        // 1. Read sensors.
        intakeSubsystem.update();

        // 2. Read the gamepad and decide what to do.
        boolean dpadUpPressed = gamepad1.dpad_up;
        boolean dpadDownPressed = gamepad1.dpad_down;

        // Turn the dial only on the loop where the button FIRST goes down.
        if (dpadUpPressed && !dpadUpWasPressed) {
            dialPercent = dialPercent + DIAL_STEP_PERCENT;
        }
        if (dpadDownPressed && !dpadDownWasPressed) {
            dialPercent = dialPercent - DIAL_STEP_PERCENT;
        }

        // Keep the dial between its lowest and highest settings.
        if (dialPercent > DIAL_MAX_PERCENT) {
            dialPercent = DIAL_MAX_PERCENT;
        }
        if (dialPercent < DIAL_MIN_PERCENT) {
            dialPercent = DIAL_MIN_PERCENT;
        }

        // Remember the D-pad for the next loop.
        dpadUpWasPressed = dpadUpPressed;
        dpadDownWasPressed = dpadDownPressed;

        // 3. Tell the intake what to do.
        // Turn the percent into the 0.0 to 1.0 number the subsystem wants (50 -> 0.50).
        intakeSubsystem.setIntakePower(dialPercent / 100.0);

        // Check REVERSE first, so clearing a jam always works.
        String activeButton;
        if (gamepad1.left_bumper) {
            intakeSubsystem.reverse();
            activeButton = "Left Bumper (REVERSE)";
        } else if (gamepad1.right_bumper) {
            intakeSubsystem.intake();
            activeButton = "Right Bumper (INTAKE)";
        } else {
            // No bumper held, so stop the intake.
            intakeSubsystem.stop();
            activeButton = "None";
        }

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Button", activeButton);
        intakeSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motor stopped.
        intakeSubsystem.stop();
    }
}
