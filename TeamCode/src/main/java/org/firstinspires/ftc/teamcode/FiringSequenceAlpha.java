package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FiringSequence;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Firing Sequence Testing
 * An iterative OpMode for trying out FiringSequence (hold one button, and the robot waits until it
 * is ready, opens the gate, feeds the balls one at a time, and closes the gate) and the PRE-REV
 * toggle. It uses gamepad1 only.
 *   FiringSequence        -> the steps (no hardware, it only gives orders)
 *   ShooterSubsystem      -> spins the flywheel and the hood wheel (isFlywheelAtSpeed() is the check)
 *   ShooterServoSubsystem -> the gate: CLOSED, OPENED, LIFTED
 *   IntakeSubsystem       -> carries the balls up the chute (one motor does the intake AND the chute)
 *   TurretSubsystem       -> turns the turret to the hive (it tracks all the time, like the TeleOp)
 *   VisionSubsystem       -> the Limelight: tx and ty of our hive's tags
 *   ShotTable             -> turns ty into a flywheel RPM
 * This OpMode owns NO hardware itself. It only reads the gamepad and passes the orders on. It does
 * NOT drive: MOVE THE ROBOT BY HAND to a spot where the camera sees the hive's tags, like
 * ShotTableAlpha. Load 4 balls into the chute first.
 *
 * GAMEPAD 1:
 *   Right Trigger (hold) -> FIRE a volley. It waits until the flywheel is at speed and the turret is
 *                           on target (and, in TABLE mode, the camera sees the hive), then it runs
 *                           the gate and the intake by itself. Let go at any time = abort: the gate
 *                           closes and the intake stops. After it finishes, let go and press again
 *                           to fire again.
 *   D-pad Up (tap)       -> PRE-REV on / off. The wheels spin without firing, so the spin-up wait is
 *                           over before you fire. It is shown big on the screen. Starts OFF.
 *   Right Bumper (hold)  -> INTAKE (pick up balls). The gate stays where it is. While a volley is
 *                           FEEDING, the sequence runs the intake instead.
 *   Left Bumper (hold)   -> REVERSE both wheels AND the intake (clearing a jam). Beats everything.
 *   X / B / Y            -> the gate by hand: X CLOSED, B OPENED, Y LIFTED (one move per press).
 *                           They work ONLY while no volley is running. (The sequence closes the
 *                           gate at the start and the end of every volley.)
 *   Back                 -> switch the speed source TABLE <-> DIAL (one switch per press).
 *                           TABLE (starts here): the flywheel speed comes from the shot table while
 *                           the camera sees the hive. A volley will NOT start with no target.
 *                           DIAL: a fixed speed, DIAL_FLYWHEEL_RPM, with no camera needed.
 *   START                -> switch alliance BLUE <-> RED (one switch per press). Starts as BLUE.
 *      (Pairing the controller on the Driver Station is Start + A. If you do that while this
 *       OpMode is running, it also switches the alliance, so check it on the screen.)
 *
 * Priority for the intake, highest first: Left Bumper (jam), the firing sequence (feeding),
 * Right Bumper (pick up).
 *
 * THE TEST ORDER (write the numbers in TeamCode/docs/biobuzz-test-and-match-log.md)
 *   1. INIT. Check "Vision State" says TARGET_VISIBLE and the alliance is right. The gate goes to
 *      CLOSED. Nothing moves.
 *   2. No balls first (the chute empty). Hold Right Trigger. Watch the screen: "Sequence step"
 *      should go WAIT_FOR_READY -> FEED -> LIFT -> DONE, and the gate CLOSED -> OPENED -> LIFTED ->
 *      CLOSED. Then let go.
 *   3. Hold Right Trigger and let go in the middle of FEED. The gate must close and the intake stop
 *      at once ("Last volley: ABORTED during FEED").
 *   4. PRE-REV: tap D-pad Up. The wheels spin. Time how long Right Trigger takes to start feeding
 *      with and without it ("Last volley: seconds to ready").
 *   5. Now with 4 balls. Hold Right Trigger. Do ALL 4 balls leave? Write down "flywheel dips" (is it
 *      about 4?) and "total seconds". If a ball is left behind, raise FEED_SECONDS in
 *      FiringSequence. (There is no LIFT step any more: it wedged the last ball, issue #63.
 *      Watch whether a ball is caught when the gate closes at the end.) To check that the intake moves the balls at all, set
 *      INTAKE_WAITS_FOR_SPEED to false in FiringSequence.
 *   6. Cover the camera lens in TABLE mode, hold Right Trigger: it must NOT start feeding, and the
 *      screen says NO TARGET.
 *   7. Hold Left Bumper while firing: the intake reverses.
 *   8. With the gate CLOSED and no volley, hold Right Bumper to pick up balls: they must stay in.
 *
 * SAFE USE: the wheels are fast. Keep hands, hair and loose clothing away from them, and from the
 * intake and the turret. Keep a hand near the Stop button.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name:           flywheel       (ShooterSubsystem, encoder plugged in)
 *   Motor name:           hood_wheel     (ShooterSubsystem, encoder plugged in)
 *   Motor name:           intake         (IntakeSubsystem)
 *   Motor name:           turret_motor   (TurretSubsystem)
 *   Servo name:           shooter_servo  (ShooterServoSubsystem)
 *   Ethernet Device name: limelight      (VisionSubsystem, Limelight 3A)
 */
@TeleOp(name = "Firing Sequence - Alpha", group = "Testing")
public class FiringSequenceAlpha extends OpMode {

    // ------------------------------------------------------------------
    // Test-only numbers
    // ------------------------------------------------------------------
    // The flywheel RPM used in DIAL mode, and in TABLE mode when the table has no answer yet. It
    // is the same number the TeleOp's speed dial starts at. It must be 1458 or lower
    // (ShooterSubsystem cuts it there).
    private static final double DIAL_FLYWHEEL_RPM = 1300.0;

    // In TABLE mode, if the camera loses the hive, keep the last table RPM for this many seconds
    // before going back to DIAL_FLYWHEEL_RPM. (The same idea as the TeleOp.)
    private static final double TABLE_RPM_HOLD_SECONDS = 1.0;

    // ShooterSubsystem treats ANY change of target RPM as "start over", and the table's answer
    // wobbles a little because ty does. So we only change the target when the wanted RPM is more
    // than this far from the one we are already using. (Otherwise the "at speed" check could keep
    // restarting and the feed would stutter.)
    private static final double RPM_CHANGE_DEADBAND = 10.0;

    // Same numbers as BiobuzzTeleOp's auto-align. We only pick a new turret target when the camera
    // says we are more than this far off (degrees), so the turret can settle.
    private static final double ALIGN_REAIM_THRESHOLD_DEGREES = 1.0;

    // "Aimed" means the camera says the hive's tx is this small (degrees) AND the turret says it is
    // on target.
    private static final double AIMED_TOLERANCE_DEGREES = 2.0;

    // Reverse power is NEGATIVE so the motors spin the other way (the same as ShotTableAlpha).
    // This is for the two shooter wheels. The intake reverses at its own power dial.
    private static final double REVERSE_POWER = -0.60;

    // The triggers give a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // true = the control map stays on the Driver Station screen while the OpMode RUNS (at the
    // bottom, under the live data). false = it is only shown before START.
    private static final boolean SHOW_CONTROLS_WHILE_RUNNING = true;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private FiringSequence firingSequence;
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TurretSubsystem turretSubsystem;
    private ShotTable shotTable;

    // PRE-REV: are the wheels being told to spin even though nobody is firing?
    private boolean preRevOn = false;

    // TABLE (true) or DIAL (false).
    private boolean useTable = true;

    // The table's last answer, and when the camera last saw the hive.
    private double lastTableRpm = 0.0;
    private boolean haveTableRpm = false;
    private ElapsedTime timeSinceTarget = new ElapsedTime();

    // The flywheel RPM we are really using, and a note on where it came from for the screen.
    private double commandedRpm = 0.0;
    private String rpmSource = "none yet";

    // During a volley's FEED and DONE steps the RPM is frozen, so the target does not move
    // while the balls are going through.
    private boolean rpmFrozen = false;
    private double frozenRpm = 0.0;

    // What the buttons were doing LAST loop, so one press counts only once.
    private boolean wasPreRevPressed = false;
    private boolean wasSourcePressed = false;
    private boolean wasStartPressed = false;
    private boolean wasGateClosedPressed = false;
    private boolean wasGateOpenedPressed = false;
    private boolean wasGateLiftedPressed = false;

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
        telemetry.addLine("Right Trigger (hold) = FIRE a volley (let go = abort)");
        telemetry.addLine("D-pad Up = PRE-REV on / off");
        telemetry.addLine("Right Bumper (hold) = INTAKE   Left Bumper (hold) = REVERSE (jam)");
        telemetry.addLine("X / B / Y = gate CLOSED / OPENED / LIFTED (only when no volley runs)");
        telemetry.addLine("Back = speed source TABLE / DIAL   START = alliance BLUE / RED");
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
     * Back switches the speed source between TABLE and DIAL.
     */
    private void checkSpeedSourceToggle() {
        boolean sourcePressed = gamepad1.back;
        if (isNewPress(sourcePressed, wasSourcePressed)) {
            useTable = !useTable;
        }
        wasSourcePressed = sourcePressed;
    }

    /*
     * D-pad Up switches PRE-REV on and off.
     */
    private void checkPreRevToggle() {
        boolean preRevPressed = gamepad1.dpad_up;
        if (isNewPress(preRevPressed, wasPreRevPressed)) {
            preRevOn = !preRevOn;
        }
        wasPreRevPressed = preRevPressed;
    }

    /*
     * Works out the flywheel RPM we WANT right now, and writes where it came from into rpmSource.
     * In TABLE mode that is the table's answer while the camera sees the hive.
     */
    private double chooseFlywheelRpm(boolean hasTarget) {
        if (hasTarget) {
            lastTableRpm = shotTable.getFlywheelRpm(visionSubsystem.getTyDegrees());
            haveTableRpm = true;
            timeSinceTarget.reset();
        }

        if (!useTable) {
            rpmSource = "DIAL (fixed speed)";
            return DIAL_FLYWHEEL_RPM;
        }
        if (hasTarget) {
            rpmSource = "TABLE";
            return lastTableRpm;
        }
        if (haveTableRpm && timeSinceTarget.seconds() <= TABLE_RPM_HOLD_SECONDS) {
            rpmSource = "TABLE (camera lost the hive, holding the last RPM)";
            return lastTableRpm;
        }
        rpmSource = "DIAL speed (TABLE mode, but no table answer yet)";
        return DIAL_FLYWHEEL_RPM;
    }

    /*
     * Turns the turret toward the hive, every loop, like the TeleOp's ALWAYS-TRACK. It is the
     * TeleOp's auto-align without the "follow the robot's spin" part, because this OpMode does not
     * drive. With no camera target it holds where it is.
     */
    private void aimTurretAtHive() {
        if (!visionSubsystem.hasTarget()) {
            // Hold the current angle (once), so the turret does not flop around, and so the
            // turret's "on target" check can become true.
            if (turretSubsystem.getState() != TurretSubsystem.TurretState.POSITION) {
                turretSubsystem.holdCurrentAngle();
            }
            return;
        }

        double txDegrees = visionSubsystem.getTxDegrees();

        // Pick a new target if the turret is not in POSITION mode yet (the align just started),
        // or if the camera says we are far enough off. Picking a new target every loop would keep
        // the turret from ever settling.
        boolean notAimingYet = turretSubsystem.getState() != TurretSubsystem.TurretState.POSITION;
        boolean farOff = Math.abs(txDegrees) > ALIGN_REAIM_THRESHOLD_DEGREES;
        if (notAimingYet || farOff) {
            // tx is positive to the RIGHT, but turret angles are positive to the LEFT, so subtract.
            turretSubsystem.setTargetAngle(turretSubsystem.getAngleDegrees() - txDegrees);
        }
    }

    /*
     * The words for "what are we still waiting for?", shown while a volley waits.
     */
    private String getWaitingText(boolean hasTarget, boolean aimedAtHive) {
        String waiting = "";
        if (!shooterSubsystem.isFlywheelAtSpeed()) {
            waiting = waiting + "flywheel not at speed   ";
        }
        if (useTable && !hasTarget) {
            waiting = waiting + "NO TARGET (the camera must see the hive)   ";
        } else if (useTable && !aimedAtHive) {
            waiting = waiting + "turret not aimed yet   ";
        } else if (!useTable && !turretSubsystem.isOnTarget()) {
            waiting = waiting + "turret not settled   ";
        }
        if (waiting.isEmpty()) {
            return "nothing, ready to fire";
        }
        return waiting;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        firingSequence = new FiringSequence();
        // The Limelight starts polling here. The shooter does not spin at INIT.
        visionSubsystem = new VisionSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        // The gate moves to CLOSED here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        // The turret does not move at INIT. It only starts reading its encoder.
        turretSubsystem = new TurretSubsystem(hardwareMap);
        shotTable = new ShotTable();

        telemetry.addData("Status", "Initialized");
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Check the camera and set the alliance and the speed source now, before the wheels can spin.
        visionSubsystem.update();
        checkAllianceToggle();
        checkSpeedSourceToggle();

        telemetry.addData("Status", "Ready to start!");
        telemetry.addData("Speed source (Back)", useTable ? "TABLE" : "DIAL");
        visionSubsystem.addTelemetry(telemetry);
        addControlsTelemetry();

        // The turret is not driven during INIT. We only read it.
        turretSubsystem.update();
        turretSubsystem.addTelemetry(telemetry);
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
        // 1. Read sensors. (The shooter, intake and turret read their own encoders in update().)
        visionSubsystem.update();
        intakeSubsystem.update();

        // 2. Read the gamepad and decide what to do.
        checkAllianceToggle();
        checkSpeedSourceToggle();
        checkPreRevToggle();

        boolean fireHeld = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean reverseHeld = gamepad1.left_bumper;
        boolean intakeHeld = gamepad1.right_bumper;

        // The manual gate buttons. We work out the presses every loop (so an old press never
        // waits around), and only use them below when the firing sequence is not in charge.
        boolean gateClosedNow = gamepad1.x;
        boolean gateOpenedNow = gamepad1.b;
        boolean gateLiftedNow = gamepad1.y;
        boolean closedNewPress = isNewPress(gateClosedNow, wasGateClosedPressed);
        boolean openedNewPress = isNewPress(gateOpenedNow, wasGateOpenedPressed);
        boolean liftedNewPress = isNewPress(gateLiftedNow, wasGateLiftedPressed);
        wasGateClosedPressed = gateClosedNow;
        wasGateOpenedPressed = gateOpenedNow;
        wasGateLiftedPressed = gateLiftedNow;

        // What speed do we want? During the feed and the end of a volley we freeze it, so
        // the target does not move while balls are going through.
        boolean hasTarget = visionSubsystem.hasTarget();
        FiringSequence.SequenceState sequenceStep = firingSequence.getState();
        boolean feedingOrLater = sequenceStep == FiringSequence.SequenceState.FEED
                || sequenceStep == FiringSequence.SequenceState.DONE;
        double wantedRpm;
        if (feedingOrLater) {
            if (!rpmFrozen) {
                frozenRpm = commandedRpm;
                rpmFrozen = true;
            }
            wantedRpm = frozenRpm;
        } else {
            rpmFrozen = false;
            wantedRpm = chooseFlywheelRpm(hasTarget);
        }
        // Do not chase tiny changes (see RPM_CHANGE_DEADBAND).
        if (commandedRpm == 0.0 || Math.abs(wantedRpm - commandedRpm) > RPM_CHANGE_DEADBAND) {
            commandedRpm = wantedRpm;
        }

        // 3. Tell the subsystems what to do, then call update().
        // The turret tracks the hive all the time.
        aimTurretAtHive();

        // The wheels spin when we are firing OR pre-revving. Reverse (a jam) beats everything.
        boolean wheelsWanted = fireHeld || preRevOn;
        String shooterControl;
        if (reverseHeld) {
            shooterSubsystem.setPower(REVERSE_POWER, REVERSE_POWER);
            shooterControl = "Left Bumper (REVERSE)";
        } else if (wheelsWanted) {
            shooterSubsystem.setTargetRpm(commandedRpm);
            if (fireHeld) {
                shooterControl = "FIRE (" + rpmSource + ")";
            } else {
                shooterControl = "PRE-REV (" + rpmSource + ")";
            }
        } else {
            shooterSubsystem.stop();
            shooterControl = "off";
        }
        shooterSubsystem.update();
        turretSubsystem.update();

        // The firing sequence. It comes AFTER the updates, so isFlywheelAtSpeed() and isOnTarget()
        // are fresh. The turret half of "ready": in TABLE mode the camera must see the hive RIGHT
        // NOW and be centered on it, and the turret must have settled. In DIAL mode there is no
        // camera to check, so the turret just has to have settled.
        boolean aimedAtHive = hasTarget
                && Math.abs(visionSubsystem.getTxDegrees()) <= AIMED_TOLERANCE_DEGREES
                && turretSubsystem.isOnTarget();
        boolean turretReady;
        if (useTable) {
            turretReady = aimedAtHive;
        } else {
            turretReady = turretSubsystem.isOnTarget();
        }
        boolean readyToStart = shooterSubsystem.isFlywheelAtSpeed() && turretReady;
        firingSequence.update(fireHeld, readyToStart, shooterSubsystem.isFlywheelAtSpeed());

        // The gate. CLOSING is GENTLE (goToClosedGently), like the TeleOp, so a single ball left in the
        // way is pushed aside instead of slammed or wedged. The sequence is in charge while a volley
        // runs. Otherwise the manual buttons work. If more than one is pressed on the same loop,
        // CLOSED wins, then OPENED, then LIFTED.
        FiringSequence.GateCommand gateCommand = firingSequence.getGateCommand();
        String gateControl;
        if (gateCommand == FiringSequence.GateCommand.CLOSE) {
            shooterServoSubsystem.goToClosedGently();
            gateControl = "firing sequence";
        } else if (gateCommand == FiringSequence.GateCommand.OPEN) {
            shooterServoSubsystem.goToOpened();
            gateControl = "firing sequence";
        } else if (closedNewPress) {
            shooterServoSubsystem.goToClosedGently();
            gateControl = "manual (X)";
        } else if (openedNewPress) {
            shooterServoSubsystem.goToOpened();
            gateControl = "manual (B)";
        } else if (liftedNewPress) {
            shooterServoSubsystem.goToLifted();
            gateControl = "manual (Y)";
        } else {
            gateControl = "manual (no press)";
        }
        // Walk a gentle close along. (Does nothing when no gentle close is going on.)
        shooterServoSubsystem.update();

        // The intake. Reverse goes first, so clearing a jam always works. Then the sequence (while
        // it is feeding). Then the driver's own intake button.
        String intakeControl;
        if (reverseHeld) {
            intakeSubsystem.reverse();
            intakeControl = "Left Bumper (REVERSE)";
        } else if (firingSequence.ownsIntake()) {
            if (firingSequence.wantsIntake()) {
                intakeSubsystem.intake();
                intakeControl = "firing sequence: FEEDING";
            } else {
                intakeSubsystem.stop();
                intakeControl = "firing sequence: waiting for the flywheel";
            }
        } else if (intakeHeld) {
            intakeSubsystem.intake();
            intakeControl = "Right Bumper (INTAKE)";
        } else {
            intakeSubsystem.stop();
            intakeControl = "off";
        }

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        if (preRevOn) {
            telemetry.addData("PRE-REV (D-pad Up)", "ON  (the wheels are spinning)");
        } else {
            telemetry.addData("PRE-REV (D-pad Up)", "off");
        }
        telemetry.addData("Speed source (Back)", useTable ? "TABLE" : "DIAL");
        telemetry.addData("Shooter control", shooterControl);
        telemetry.addData("Gate control", gateControl);
        telemetry.addData("Intake control", intakeControl);

        if (firingSequence.getState() == FiringSequence.SequenceState.WAIT_FOR_READY) {
            telemetry.addData("Waiting for", getWaitingText(hasTarget, aimedAtHive));
        }

        telemetry.addLine("--- SHOT ---");
        telemetry.addData("Flywheel RPM we are using", "%.0f", commandedRpm);
        if (hasTarget) {
            telemetry.addData("ty now", "%.2f deg", visionSubsystem.getTyDegrees());
            if (!shotTable.isInRange(visionSubsystem.getTyDegrees())) {
                telemetry.addLine("WARNING: ty is OUTSIDE the table. It is using the nearest row.");
            }
        } else {
            telemetry.addData("ty now", "NO TARGET (check the alliance and that the camera sees the tags)");
        }
        telemetry.addData("Flywheel at speed (the volley's check)", shooterSubsystem.isFlywheelAtSpeed());
        telemetry.addData("Hood wheel error (actual - target, RPM)", "%.0f",
                shooterSubsystem.getHoodWheelRpm() - shooterSubsystem.getTargetHoodWheelRpm());
        telemetry.addData("Both wheels at speed (info only)", shooterSubsystem.isAtSpeed());
        telemetry.addData("Turret on target", turretSubsystem.isOnTarget());

        firingSequence.addTelemetry(telemetry);
        shooterSubsystem.addTelemetry(telemetry);
        shooterServoSubsystem.addTelemetry(telemetry);
        intakeSubsystem.addTelemetry(telemetry);
        turretSubsystem.addTelemetry(telemetry);
        visionSubsystem.addTelemetry(telemetry);

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
        // Always leave the motors stopped, the gate closed, and stop polling the camera.
        shooterSubsystem.stop();
        intakeSubsystem.stop();
        shooterServoSubsystem.goToClosed();
        turretSubsystem.stop();
        visionSubsystem.stop();
    }
}
