package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Auto - Shoot and Park, Alpha 1
 * Shoots our preloaded POLLEN at the hive, then drives into our LOADING ZONE (rungs A-3 + A-1 in
 * TeamCode/docs/biobuzz-auto-strategy.md). It is LeaveAndParkAuto plus a firing sequence.
 *
 *   1. FIND_HIVE         : the turret turns until the camera is locked on the hive, and the
 *                          flywheel starts spinning at PRE_REV_FLYWHEEL_RPM so it is already
 *                          warm. As soon as the camera is locked, the shot RPM comes from the
 *                          shot table (from ty).
 *   2. WAIT_FOR_SPEED    : the flywheel AND the hood wheel get to the target RPM and stay there
 *                          (ShooterSubsystem.isAtSpeed()), and the turret is on target.
 *   3. GATE_OPEN         : the gate goes CLOSED -> OPENED and waits GATE_OPEN_SECONDS.
 *   4. GATE_LIFT         : the gate goes to LIFTED and waits GATE_LIFT_SECONDS.
 *      Then the gate goes back to CLOSED and the shooter stops.
 *   5. DRIVE_TO_WAYPOINT : slide to the right of the hive (so we do not clip it).
 *   6. DRIVE_TO_PARK     : drive down the field into the LOADING ZONE.
 *   7. DONE              : the robot holds its spot.
 *
 * IF THE SHOT CANNOT HAPPEN, SKIP IT AND PARK (we never give up the park points):
 *   - the camera is not locked on the hive after LOCK_TIMEOUT_SECONDS, or
 *   - the wheels are not at speed after SPIN_UP_TIMEOUT_SECONDS.
 *   The screen says which one happened ("Shot skipped").
 *
 * WHY "AT SPEED" CHECKS BOTH WHEELS
 *   ShooterSubsystem.isAtSpeed() is only true when the flywheel AND the hood wheel are both
 *   within tolerance of their targets for a short hold time. We use it as it is. The hood wheel
 *   sets the ball's spin, so a shot with the hood wheel off its speed would fly differently.
 *
 * WHAT THIS OPMODE DOES NOT DO
 *   - It does not run the intake. The 4 preloaded balls must already be in the shooter, held
 *     back by the gate, like the TeleOp. [CONFIRM] on the robot: watch whether OPENED then LIFTED
 *     really releases the balls we want. If not, the sequence needs another step (intake or a
 *     second pass). That is a design change, so ask the coach.
 *   - It does not move while shooting. The robot shoots from its start spot.
 *
 * Pedro's Follower owns the drive motors and the Pinpoint, so this OpMode does NOT use
 * DriveSubsystem or PinpointSubsystem. The shooter, gate, turret, camera and shot table are the
 * same classes the TeleOp uses.
 *
 * WHERE THE ROBOT STARTS (BLUE, the default)   (all the numbers are in pedro/FieldConstants.java)
 *   Tile D6, in the middle of the tile, with the robot's BACK against the TOP wall, facing DOWN
 *   the field toward the hive (the intake and the turret point at the hive). Measure from the
 *   LEFT wall to the middle of the robot: about 82.5 inches.
 *   The robot must not touch a flower, no part of it may be in the LOADING ZONE, and it must hold
 *   exactly 4 POLLEN.
 *   RED (press B in INIT) is the same plan turned 180 degrees: tile C1, back against the BOTTOM
 *   wall, facing UP. Red has not been driven yet, so check it slowly the first time.
 *
 * BEFORE YOU RUN IT (once per robot)
 *   - The turret must have been pointing STRAIGHT AHEAD when the robot was powered on (the turret
 *     home switch is off in code, so 0 degrees is where it pointed at power-up), and it should be
 *     pointing roughly straight ahead now.
 *   - The Limelight needs about 30 seconds after power-up before it sees anything.
 *   - The driver station config must have the "limelight" Ethernet device (like the TeleOp).
 *   - To drive, Pedro must be tuned (TeamCode/docs/pedro-setup-and-auto-guide.md) and
 *     FORESIGHT_TUNED set to true. If it is NOT tuned, this OpMode still does the SHOT but does
 *     not drive, so you can test the firing sequence on its own.
 *   - Test the shot with the robot on its start spot before you trust the park: check that the
 *     camera sees the hive from there (see the INIT screen), and that the ty on the screen is
 *     near a row of the shot table.
 *
 * HOW TO RUN IT
 *   1. Put the robot on its start spot, with 4 balls loaded, and leave it STILL (INIT
 *      recalibrates the Pinpoint's gyro).
 *   2. Select "Auto - Shoot and Park - Alpha 1" and press INIT. Wait for "Pinpoint ready".
 *   3. Check the INIT screen: ALLIANCE (gamepad1 X = BLUE, B = RED), camera connected, "Hive
 *      visible: YES" and the table RPM for the start spot.
 *   4. Press START. (The 30 second timer should be ON, like in a match.)
 *
 * WHAT TO TUNE (all at the top of this file, change one at a time)
 *   GATE_OPEN_SECONDS and GATE_LIFT_SECONDS: practice until the balls leave the way you want, then
 *   leave them alone for matches. PRE_REV_FLYWHEEL_RPM, LOCK_TIMEOUT_SECONDS, SPIN_UP_TIMEOUT_SECONDS.
 *
 * Run it 5 times in a row before you trust it, and write each run in
 * TeamCode/docs/biobuzz-test-and-match-log.md. Once it is solid, change group = "Testing" to
 * "Competition" below.
 */
@Autonomous(name = "Auto - Shoot and Park - Alpha 1", group = "Testing",
        preselectTeleOp = "TeleOp - Alpha 10")
public class ShootAndParkAuto extends OpMode {

    // ------------------------------------------------------------------
    // Settings (change these to change the Auto)
    // ------------------------------------------------------------------
    // How long the gate stays OPENED, then how long it stays LIFTED, in seconds. Practice with
    // these until the shot works, then keep them the same for matches.
    private static final double GATE_OPEN_SECONDS = 0.5;
    private static final double GATE_LIFT_SECONDS = 0.5;

    // The flywheel speed to warm up at while the camera is still finding the hive. It is the same
    // number the TeleOp's speed dial starts at. The shot table's RPM takes over as soon as the
    // camera is locked. It must be 1458 or lower (ShooterSubsystem cuts it there).
    private static final double PRE_REV_FLYWHEEL_RPM = 1300.0;

    // How long to wait for the camera to lock onto the hive before skipping the shot.
    private static final double LOCK_TIMEOUT_SECONDS = 3.0;

    // How long to wait for the wheels to reach speed (after the camera locks) before skipping.
    private static final double SPIN_UP_TIMEOUT_SECONDS = 4.0;

    // The most time one drive step is allowed to take. (Same idea as in LeaveAndParkAuto.)
    private static final double LEG_TIMEOUT_SECONDS = 6.0;

    // Same numbers as BiobuzzTeleOp's auto-align. We only pick a new turret target when the camera
    // says we are more than this far off, so the turret can settle (degrees).
    private static final double ALIGN_REAIM_THRESHOLD_DEGREES = 1.0;

    // The camera counts as LOCKED when the hive's tx is this small (degrees) and the turret says
    // it is on target.
    private static final double LOCKED_TOLERANCE_DEGREES = 2.0;

    // ------------------------------------------------------------------
    // The steps of this Auto. The robot is always in exactly one of these.
    // ------------------------------------------------------------------
    private enum AutoState {
        FIND_HIVE,
        WAIT_FOR_SPEED,
        GATE_OPEN,
        GATE_LIFT,
        DRIVE_TO_WAYPOINT,
        DRIVE_TO_PARK,
        DONE
    }

    // ------------------------------------------------------------------
    // Things this OpMode uses
    // ------------------------------------------------------------------
    private Follower follower;
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShotTable shotTable;

    private VisionSubsystem.Alliance alliance = VisionSubsystem.Alliance.BLUE;

    private Pose startPose;
    private Pose waypointPose;
    private Pose parkPose;

    // The two paths are built when the alliance is chosen (INIT), never inside loop().
    private Path waypointPath;
    private Path parkPath;

    private AutoState autoState = AutoState.DONE;

    // Times the current step (waiting for the camera, the wheels, or one gate move).
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Times one drive step, so a step that never settles cannot use up the whole Auto.
    private final ElapsedTime legTimer = new ElapsedTime();

    // The flywheel RPM the shot table gave us when the camera locked. It is saved so the target
    // does not wobble with the camera's ty while the wheels are getting to speed.
    private double shotFlywheelRpm = 0.0;

    // Why the shot was skipped, or "" if it was not. Shown on the screen.
    private String skipReason = "";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Create the Follower. This recalibrates the Pinpoint, so keep the robot still.
        follower = Constants.create(hardwareMap);

        // Each subsystem gets its own hardware from the hardware map.
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        // The gate moves to CLOSED here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);
        // The turret does not move at INIT. It only starts reading its encoder.
        turretSubsystem = new TurretSubsystem(hardwareMap);
        // The Limelight starts polling here. It needs about 30 seconds to boot after power-up.
        visionSubsystem = new VisionSubsystem(hardwareMap);
        shotTable = new ShotTable();

        // BLUE is the default. This builds the poses and paths and tells Pedro where we start.
        applyAlliance(VisionSubsystem.Alliance.BLUE);

        telemetry.addLine("Pinpoint ready. Keep the robot still, then press START.");
        addStatusTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // The Follower is idle here, so this only reads the Pinpoint and keeps the wheels stopped.
        follower.update();

        // Read the camera now, so we can check before START that it sees the hive.
        visionSubsystem.update();

        // gamepad1 X picks BLUE, gamepad1 B picks RED. Nothing happens if it is already chosen.
        if (gamepad1.x && alliance != VisionSubsystem.Alliance.BLUE) {
            applyAlliance(VisionSubsystem.Alliance.BLUE);
        } else if (gamepad1.b && alliance != VisionSubsystem.Alliance.RED) {
            applyAlliance(VisionSubsystem.Alliance.RED);
        }

        addStatusTelemetry();
        addPoseTelemetry();
        addInitCameraTelemetry();

        // The turret is NOT driven during INIT (the rules say nothing may move). We only read it.
        turretSubsystem.update();
        turretSubsystem.addTelemetry(telemetry);
        shooterServoSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        // The rules say the robot has not moved since INIT, so it is on its start spot. Tell
        // Pedro that again, in case the robot was nudged while the drive team was setting up.
        follower.setPose(startPose);
        follower.update();

        // Start looking for the hive and warming up the flywheel right away.
        skipReason = "";
        autoState = AutoState.FIND_HIVE;
        stateTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors. Pedro reads the Pinpoint and sets the wheel powers. It must run
        //    every loop, or the robot does not move (or stops steering).
        follower.update();
        visionSubsystem.update();

        // 2. Decide what to do next, and tell the subsystems.
        switch (autoState) {
            case FIND_HIVE:
                // Warm the flywheel up while we look. The turret turns until the camera is centered
                // on the hive.
                shooterSubsystem.setTargetRpm(PRE_REV_FLYWHEEL_RPM);
                aimTurretAtHive();

                if (isHiveLocked()) {
                    // Locked. Ask the shot table how fast to shoot from here, and keep that number.
                    shotFlywheelRpm = shotTable.getFlywheelRpm(visionSubsystem.getTyDegrees());
                    shooterSubsystem.setTargetRpm(shotFlywheelRpm);
                    stateTimer.reset();
                    autoState = AutoState.WAIT_FOR_SPEED;
                } else if (stateTimer.seconds() >= LOCK_TIMEOUT_SECONDS) {
                    skipShotAndPark("the camera never locked onto the hive");
                }
                break;

            case WAIT_FOR_SPEED:
                // Same number every loop is safe: ShooterSubsystem ignores a repeat request.
                shooterSubsystem.setTargetRpm(shotFlywheelRpm);
                aimTurretAtHive();

                // The firing gate: BOTH wheels at speed (and held there) AND the turret on target.
                if (shooterSubsystem.isAtSpeed() && turretSubsystem.isOnTarget()) {
                    shooterServoSubsystem.goToOpened();
                    stateTimer.reset();
                    autoState = AutoState.GATE_OPEN;
                } else if (stateTimer.seconds() >= SPIN_UP_TIMEOUT_SECONDS) {
                    skipShotAndPark("the wheels never reached speed");
                }
                break;

            case GATE_OPEN:
                // The gate is OPENED. Wait, then lift.
                if (stateTimer.seconds() >= GATE_OPEN_SECONDS) {
                    shooterServoSubsystem.goToLifted();
                    stateTimer.reset();
                    autoState = AutoState.GATE_LIFT;
                }
                break;

            case GATE_LIFT:
                // The gate is LIFTED. Wait, then close it, stop the shooter and go park.
                if (stateTimer.seconds() >= GATE_LIFT_SECONDS) {
                    shooterServoSubsystem.goToClosed();
                    shooterSubsystem.stop();
                    startDriving();
                }
                break;

            case DRIVE_TO_WAYPOINT:
                // isBusy() stays true until the robot has arrived and settled.
                if (!follower.isBusy() || legTimer.seconds() >= LEG_TIMEOUT_SECONDS) {
                    legTimer.reset();
                    follower.follow(parkPath);
                    autoState = AutoState.DRIVE_TO_PARK;
                }
                break;

            case DRIVE_TO_PARK:
                if (!follower.isBusy() || legTimer.seconds() >= LEG_TIMEOUT_SECONDS) {
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                // Nothing left to do. The Follower keeps holding the last pose.
                break;
        }

        // The shooter and the turret need update() once per loop, AFTER they get their commands.
        shooterSubsystem.update();
        turretSubsystem.update();

        // 3. Telemetry
        addStatusTelemetry();
        telemetry.addData("State", autoState);
        if (!skipReason.isEmpty()) {
            telemetry.addData("Shot skipped because", skipReason);
        }
        addShotTelemetry();
        addPoseTelemetry();
        telemetry.addData("Distance to end of path (in)", "%.2f", follower.distanceToEndpoint());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Put the Follower in IDLE and tell the drivetrain to stop right now. (follower.stop()
        // only changes the mode. The wheels are zeroed by the next update(), and there is none.)
        follower.stop();
        follower.drivetrain.stop();

        shooterSubsystem.stop();
        // The turret floats. The hub keeps its encoder count, so the TeleOp knows its angle.
        turretSubsystem.stop();
        visionSubsystem.stop();
    }

    // ------------------------------------------------------------------
    // The steps
    // ------------------------------------------------------------------
    /*
     * Turns the turret toward the hive. Call it every loop while we are looking and waiting.
     * It is the TeleOp's auto-align without the "follow the robot's spin" part, because the robot
     * does not move while it shoots.
     */
    private void aimTurretAtHive() {
        if (!visionSubsystem.hasTarget()) {
            // The camera does not see the hive. Leave the turret where it is.
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
            double newTargetAngle = turretSubsystem.getAngleDegrees() - txDegrees;
            turretSubsystem.setTargetAngle(newTargetAngle);
        }
    }

    /*
     * True when the camera sees the hive, the hive is nearly centered in the view, and the turret
     * says it is on target.
     */
    private boolean isHiveLocked() {
        return visionSubsystem.hasTarget()
                && Math.abs(visionSubsystem.getTxDegrees()) <= LOCKED_TOLERANCE_DEGREES
                && turretSubsystem.isOnTarget();
    }

    /*
     * Give up on the shot (and say why), stop the shooter, and go park.
     */
    private void skipShotAndPark(String reason) {
        skipReason = reason;
        shooterSubsystem.stop();
        shooterServoSubsystem.goToClosed();
        startDriving();
    }

    /*
     * The shooting part is over (or skipped). Start the first drive step.
     */
    private void startDriving() {
        // Hold the turret where it is while we drive, so it does not swing around.
        turretSubsystem.holdCurrentAngle();

        if (!Constants.FORESIGHT_TUNED) {
            // Do not drive on another robot's braking numbers. The shot (if any) is already done.
            autoState = AutoState.DONE;
            return;
        }

        legTimer.reset();
        follower.follow(waypointPath);
        autoState = AutoState.DRIVE_TO_WAYPOINT;
    }

    // ------------------------------------------------------------------
    // Alliance
    // ------------------------------------------------------------------
    /*
     * Picks the alliance: tells the camera which hive is ours, builds the three poses and the two
     * paths, and tells Pedro where the robot is standing. Only call it in INIT, never in loop().
     */
    private void applyAlliance(VisionSubsystem.Alliance newAlliance) {
        alliance = newAlliance;
        visionSubsystem.setAlliance(alliance);

        startPose = FieldConstants.startPose(alliance);
        waypointPose = FieldConstants.waypointPose(alliance);
        parkPose = FieldConstants.parkPose(alliance);

        // A path is a line between poses. ".constant(startPose)" means: keep facing the heading
        // of startPose the whole way, so the robot never turns. (It slides sideways instead.)
        waypointPath = Paths.line(startPose, waypointPose).constant(startPose);
        parkPath = Paths.line(waypointPose, parkPose).constant(startPose);

        // Tell Pedro where the robot is standing right now.
        follower.setPose(startPose);
    }

    // ------------------------------------------------------------------
    // Telemetry helpers
    // ------------------------------------------------------------------
    private void addStatusTelemetry() {
        telemetry.addData("ALLIANCE (gamepad1 X = BLUE, B = RED)", alliance);
        if (alliance == VisionSubsystem.Alliance.BLUE) {
            telemetry.addLine("Start: tile D6, back to the TOP wall, facing DOWN");
        } else {
            telemetry.addLine("Start: tile C1, back to the BOTTOM wall, facing UP");
        }

        if (Constants.FORESIGHT_TUNED) {
            telemetry.addData("Foresight", "tuned, OK to drive");
        } else {
            telemetry.addLine("!!! NOT TUNED: this OpMode will shoot but NOT drive. !!!");
            telemetry.addLine("Tune Foresight (see the Pedro guide), paste the numbers into");
            telemetry.addLine("pedro/Constants.java, then set FORESIGHT_TUNED = true.");
        }
    }

    // Checks to read BEFORE START: does the camera see the hive from the start spot, and what
    // would the shot table say?
    private void addInitCameraTelemetry() {
        telemetry.addData("Camera connected", visionSubsystem.isCameraConnected());
        if (visionSubsystem.hasTarget()) {
            double tyDegrees = visionSubsystem.getTyDegrees();
            telemetry.addData("Hive visible", "YES (ty %.1f)", tyDegrees);
            telemetry.addData("Table RPM from here", "%.0f", shotTable.getFlywheelRpm(tyDegrees));
            if (!shotTable.isInRange(tyDegrees)) {
                telemetry.addLine("Warning: ty is OUTSIDE the shot table. It uses the nearest row.");
            }
        } else {
            telemetry.addData("Hive visible", "NO (the Limelight needs ~30 s after power-up)");
        }
    }

    // What the shot is doing right now (while the Auto runs).
    private void addShotTelemetry() {
        telemetry.addData("Hive visible", visionSubsystem.hasTarget());
        telemetry.addData("tx (deg)", "%.1f", visionSubsystem.getTxDegrees());
        telemetry.addData("Turret on target", turretSubsystem.isOnTarget());
        telemetry.addData("Flywheel RPM (target / actual)", "%.0f / %.0f",
                shooterSubsystem.getTargetFlywheelRpm(), shooterSubsystem.getFlywheelRpm());
        telemetry.addData("Hood wheel RPM (target / actual)", "%.0f / %.0f",
                shooterSubsystem.getTargetHoodWheelRpm(), shooterSubsystem.getHoodWheelRpm());
        telemetry.addData("At speed (both wheels)", shooterSubsystem.isAtSpeed());
        telemetry.addData("Gate", shooterServoSubsystem.getState());
    }

    private void addPoseTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pose X (in)", "%.2f", pose.x());
        telemetry.addData("Pose Y (in)", "%.2f", pose.y());
        telemetry.addData("Pose heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
    }
}
