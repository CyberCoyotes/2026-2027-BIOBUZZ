package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Shot Table Testing
 * An iterative OpMode for MEASURING the shot table, and then CHECKING it, using gamepad1.
 *   VisionSubsystem  -> reads the Limelight: the ty of our hive's AprilTags
 *   ShooterSubsystem -> spins the flywheel and the hood wheel
 *   IntakeSubsystem  -> carries the balls up the chute to the shooter
 *   ShotTable        -> turns a ty into a flywheel RPM
 * This OpMode owns NO hardware itself. It only reads the gamepad and tells the subsystems what
 * to do. It does NOT drive, move the turret, or move the gate:
 *   - MOVE THE ROBOT BY HAND to each spot on the field, with the turret left straight ahead.
 *     Point the robot so the camera can see the hive's tags.
 *   - LOAD the balls into the intake by hand. Holding a shoot trigger runs the intake too.
 *   - The gate servo is NOT touched. It stays wherever it was. If the gate has to be open
 *     for a ball to leave the shooter, open it first (ShooterRPMAlpha or the TeleOp).
 *
 * THE INTAKE FEEDS ONLY WHEN THE WHEELS ARE READY
 *   Holding a shoot trigger spins the wheels first. The intake starts only when "Shooter State"
 *   says READY (both wheels at speed). A ball slows the wheels, the state drops back to
 *   SPINNING_UP, and the intake pauses until the wheels recover. So the balls go one at a time,
 *   each at the right speed. That keeps the measurements honest. To run the intake the moment
 *   you pull the trigger instead, set INTAKE_WAITS_FOR_SPEED to false.
 *
 * PART 1: MEASURE THE TABLE (one row per spot)
 *   1. Pick a spot. Move the robot there. Check "Vision State" says TARGET_VISIBLE.
 *   2. HOLD the Right Trigger. The wheels spin at the DIAL speed (flywheel RPM; the hood wheel
 *      is that x the ratio, set in ShooterSubsystem). Once READY, the intake feeds the balls.
 *   3. Watch where each ball goes. Did it score?
 *   4. D-pad Up / Down changes the dial. Repeat until you find the SLOWEST speed that scores
 *      reliably from that spot (a slower shot is easier on the wheels and the batteries).
 *   5. Press A. That saves one row: the spot's ty and the dial RPM. Saved rows stay on the screen.
 *   6. Go to the next spot. Aim for 4 to 6 spots, from near the hive to far from it.
 *   7. When you are done, type the saved rows into the two lists at the top of ShotTable.
 *      (Nothing is saved after the OpMode stops, so copy them down first.)
 *
 * PART 2: CHECK THE TABLE
 *   Go to a spot (a NEW one is a better test than a spot you measured). HOLD the Left Trigger.
 *   The wheels spin at whatever speed the table gives for the ty right now. Does it score?
 *   If a spot is off, change that row in ShotTable and test it again.
 *
 * GAMEPAD 1:
 *   Right Trigger (hold) -> shoot at the DIAL speed
 *   Left Trigger  (hold) -> shoot at the TABLE speed (from the ty right now)
 *   (Both triggers run the intake too, once the wheels are READY.)
 *   D-pad Up / Down      -> dial +/- DIAL_STEP_RPM (one change per press)
 *   A                    -> SAVE a row (current ty + dial RPM)
 *   B                    -> clear the saved rows
 *   Left Bumper (hold)   -> reverse BOTH wheels AND the intake (clearing a jam). Beats everything.
 *   START                -> switch alliance BLUE <-> RED (one switch per press). Starts as BLUE.
 *      (Pairing the controller on the Driver Station is Start + A. If you do that while this
 *       OpMode is running, it also switches the alliance and saves a row, so check both.)
 *   No trigger held -> both wheels coast to a stop and the intake stops.
 *
 * Priority, highest first: Left Bumper (jam clearing), Right Trigger (dial), Left Trigger (table).
 *
 * SAFE USE: the wheels are fast. Keep hands, hair and loose clothing away from them, and from
 * the intake. Keep a hand near the Stop button.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name:           flywheel     (ShooterSubsystem, encoder plugged in)
 *   Motor name:           hood_wheel   (ShooterSubsystem, encoder plugged in)
 *   Motor name:           intake       (IntakeSubsystem)
 *   Ethernet Device name: limelight    (VisionSubsystem, Limelight 3A)
 */
@TeleOp(name = "Shot Table - Alpha", group = "Testing")
public class ShotTableAlpha extends OpMode {

    // ------------------------------------------------------------------
    // Test-only numbers
    // ------------------------------------------------------------------
    // The dial is the FLYWHEEL RPM. It starts at 1300, the speed that looked about right by eye.
    // One D-pad press moves it 25 RPM, so a new spot can be tuned finely. (The first rows in
    // ShotTable were found in 50 RPM steps, so they are only good to about +/- 50.)
    // The flywheel motor is the goBILDA 1620 RPM one, and the ShooterSubsystem will not take it
    // above FLYWHEEL_MAX_TARGET_RPM (1458). So the dial stops at 1450. Raise DIAL_MAX_RPM only if
    // that limit changes. (It was 2700 when the flywheel was a 6000 RPM motor.)
    private static final int DIAL_START_RPM = 1300;
    private static final int DIAL_STEP_RPM = 25;
    private static final int DIAL_MIN_RPM = 500;
    private static final int DIAL_MAX_RPM = 1450;

    // How many rows A can save. The screen has only so much room. 8 is more than the table needs.
    private static final int MAX_SAVED_ROWS = 8;

    // true  -> the intake runs only while the shooter says READY (both wheels at speed), so each
    //          ball is fed into wheels that are at the speed you are measuring. Use this to
    //          collect table rows.
    // false -> the intake runs the whole time a shoot trigger is held, even while the wheels are
    //          still spinning up. The first ball may leave slow.
    private static final boolean INTAKE_WAITS_FOR_SPEED = true;

    // Reverse power is NEGATIVE so the motors spin the other way (the same as ShooterRPMAlpha).
    // This is for the two shooter wheels. The intake reverses at its own power dial.
    private static final double REVERSE_POWER = -0.60;

    // The measured ratio (hood RPM / flywheel RPM) is only worth showing when the flywheel is
    // spinning faster than this. Below it the division gives silly numbers.
    private static final double MIN_RPM_FOR_MEASURED_RATIO = 300.0;

    // The triggers give a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ShotTable shotTable;

    // The dial: the flywheel RPM that Right Trigger shoots at.
    private int dialRpm = DIAL_START_RPM;

    // The saved rows. Two lists that line up, like the lists in ShotTable.
    private double[] savedTy = new double[MAX_SAVED_ROWS];
    private int[] savedRpm = new int[MAX_SAVED_ROWS];
    private int savedRowCount = 0;

    // A short message about the last A or B press, shown on the screen.
    private String saveMessage = "none yet";

    // What the buttons were doing LAST loop, so one press counts only once.
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;
    private boolean wasSavePressed = false;
    private boolean wasClearPressed = false;
    private boolean wasStartPressed = false;

    /*
     * True only on the loop where a button goes from "not pressed" to "pressed".
     */
    private boolean isNewPress(boolean pressedNow, boolean pressedLastLoop) {
        return pressedNow && !pressedLastLoop;
    }

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Right Trigger (hold) = shoot at the DIAL speed (+ intake when READY)");
        telemetry.addLine("Left Trigger (hold) = shoot at the TABLE speed (+ intake when READY)");
        telemetry.addLine("D-pad Up / Down = dial +/- " + DIAL_STEP_RPM + " RPM");
        telemetry.addLine("A = SAVE a row (ty + dial RPM)   B = clear saved rows");
        telemetry.addLine("Left Bumper (hold) = REVERSE both wheels + intake (jam)");
        telemetry.addLine("START = switch alliance (BLUE / RED)");
    }

    /*
     * START switches the alliance. Works before and after the match starts.
     */
    private void checkAllianceToggle() {
        boolean startPressed = gamepad1.start;
        if (isNewPress(startPressed, wasStartPressed)) {
            if (visionSubsystem.getAlliance() == VisionSubsystem.Alliance.BLUE) {
                visionSubsystem.setAlliance(VisionSubsystem.Alliance.RED);
            } else {
                visionSubsystem.setAlliance(VisionSubsystem.Alliance.BLUE);
            }
        }
        wasStartPressed = startPressed;
    }

    /*
     * D-pad Up / Down moves the dial. The dial stays between DIAL_MIN_RPM and DIAL_MAX_RPM.
     */
    private void checkDial() {
        boolean upPressed = gamepad1.dpad_up;
        boolean downPressed = gamepad1.dpad_down;

        if (isNewPress(upPressed, wasDpadUpPressed)) {
            dialRpm = dialRpm + DIAL_STEP_RPM;
        }
        if (isNewPress(downPressed, wasDpadDownPressed)) {
            dialRpm = dialRpm - DIAL_STEP_RPM;
        }
        if (dialRpm > DIAL_MAX_RPM) {
            dialRpm = DIAL_MAX_RPM;
        }
        if (dialRpm < DIAL_MIN_RPM) {
            dialRpm = DIAL_MIN_RPM;
        }

        wasDpadUpPressed = upPressed;
        wasDpadDownPressed = downPressed;
    }

    /*
     * A saves a row and B clears them. A needs a target, because without one ty is just zero.
     */
    private void checkSaveButtons() {
        boolean savePressed = gamepad1.a;
        boolean clearPressed = gamepad1.b;

        if (isNewPress(savePressed, wasSavePressed)) {
            if (!visionSubsystem.hasTarget()) {
                saveMessage = "NOT saved - no target in view";
            } else if (savedRowCount >= MAX_SAVED_ROWS) {
                saveMessage = "NOT saved - full (press B to clear)";
            } else {
                savedTy[savedRowCount] = visionSubsystem.getTyDegrees();
                savedRpm[savedRowCount] = dialRpm;
                savedRowCount = savedRowCount + 1;
                saveMessage = "saved row " + savedRowCount;
            }
        }
        if (isNewPress(clearPressed, wasClearPressed)) {
            savedRowCount = 0;
            saveMessage = "cleared";
        }

        wasSavePressed = savePressed;
        wasClearPressed = clearPressed;
    }

    /*
     * Shows the saved rows, ready to copy into the lists in ShotTable. Sorting them from
     * smallest ty to biggest is up to you (the spots do not have to be saved in order).
     */
    private void addSavedRowsTelemetry() {
        telemetry.addLine("--- SAVED ROWS (copy into ShotTable) ---");
        telemetry.addData("Last save", saveMessage);
        for (int row = 0; row < savedRowCount; row = row + 1) {
            telemetry.addLine(String.format("row %d:  ty %.1f   RPM %d",
                    row + 1, savedTy[row], savedRpm[row]));
        }
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The Limelight starts polling here. The shooter does not spin at INIT.
        visionSubsystem = new VisionSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shotTable = new ShotTable();

        telemetry.addData("Status", "Initialized");
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Check the camera and set the alliance now, before the wheels can spin.
        visionSubsystem.update();
        checkAllianceToggle();

        telemetry.addData("Status", "Ready to start!");
        visionSubsystem.addTelemetry(telemetry);
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
        // 1. Read sensors. (The shooter and the intake read their own encoders inside update().)
        visionSubsystem.update();
        intakeSubsystem.update();

        // 2. Read the gamepad and decide what to do.
        checkAllianceToggle();
        checkDial();
        checkSaveButtons();

        boolean reverseHeld = gamepad1.left_bumper;
        boolean dialShotHeld = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean tableShotHeld = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;

        // The table's answer for the ty right now. Only meaningful when there is a target.
        boolean hasTarget = visionSubsystem.hasTarget();
        double tableRpm = 0.0;
        if (hasTarget) {
            tableRpm = shotTable.getFlywheelRpm(visionSubsystem.getTyDegrees());
        }

        // 3. Tell the shooter what to do, then call update().
        // Check from HIGHEST priority to LOWEST. The first one that is true wins.
        // shooting is true when a shoot trigger is held AND the wheels are being told to spin.
        // The intake uses it below.
        String shooterControl;
        boolean shooting = false;
        if (reverseHeld) {
            shooterSubsystem.setPower(REVERSE_POWER, REVERSE_POWER);
            shooterControl = "Left Bumper (REVERSE)";
        } else if (dialShotHeld) {
            shooterSubsystem.setTargetRpm(dialRpm);
            shooting = true;
            shooterControl = "Right Trigger (DIAL " + dialRpm + " RPM)";
        } else if (tableShotHeld && hasTarget) {
            shooterSubsystem.setTargetRpm(tableRpm);
            shooting = true;
            shooterControl = String.format("Left Trigger (TABLE %.0f RPM)", tableRpm);
        } else if (tableShotHeld) {
            // No target means no ty, so there is no table answer. Stay stopped instead of guessing.
            shooterSubsystem.stop();
            shooterControl = "Left Trigger (NO TARGET - stopped)";
        } else {
            // Nothing held, so stop both motors. They coast down.
            shooterSubsystem.stop();
            shooterControl = "None";
        }
        shooterSubsystem.update();

        // The intake. This comes AFTER shooterSubsystem.update(), because isAtSpeed() is only
        // fresh once the shooter has read its encoders this loop. Reverse goes first, so
        // clearing a jam always works.
        String intakeControl;
        if (reverseHeld) {
            intakeSubsystem.reverse();
            intakeControl = "Left Bumper (REVERSE)";
        } else if (shooting && INTAKE_WAITS_FOR_SPEED && !shooterSubsystem.isAtSpeed()) {
            // A shoot trigger is held, but the wheels are not at speed yet. Hold the balls back.
            intakeSubsystem.stop();
            intakeControl = "waiting for the wheels (not READY)";
        } else if (shooting) {
            intakeSubsystem.intake();
            intakeControl = "FEEDING";
        } else {
            intakeSubsystem.stop();
            intakeControl = "None";
        }

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Shooter Control", shooterControl);
        telemetry.addData("Intake Control", intakeControl);

        telemetry.addLine("--- SHOT ---");
        telemetry.addData("Dial (flywheel RPM)", dialRpm);
        if (hasTarget) {
            telemetry.addData("ty now", "%.2f deg", visionSubsystem.getTyDegrees());
            telemetry.addData("Table says", "%.0f flywheel RPM  (table has %d rows)",
                    tableRpm, shotTable.getRowCount());
            if (!shotTable.isInRange(visionSubsystem.getTyDegrees())) {
                telemetry.addLine("WARNING: ty is OUTSIDE the table. It is using the nearest row.");
            }
        } else {
            telemetry.addData("ty now", "NO TARGET (check the alliance and that the camera sees the tags)");
        }

        // If the hood wheel hit the speed cap, this number comes out lower than the ratio you
        // set in ShooterSubsystem. Then the dial is too high.
        if (shooterSubsystem.getFlywheelRpm() > MIN_RPM_FOR_MEASURED_RATIO) {
            telemetry.addData("Measured ratio (hood / flywheel)", "%.2f",
                    shooterSubsystem.getHoodWheelRpm() / shooterSubsystem.getFlywheelRpm());
        }

        shooterSubsystem.addTelemetry(telemetry);
        intakeSubsystem.addTelemetry(telemetry);
        visionSubsystem.addTelemetry(telemetry);
        addSavedRowsTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motors stopped, and stop polling the camera.
        shooterSubsystem.stop();
        intakeSubsystem.stop();
        visionSubsystem.stop();
    }
}
