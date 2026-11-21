package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.AutoHandoff;
import org.firstinspires.ftc.teamcode.subsystems.FlowerPickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * BiobuzzAuto: the ONE Auto that all 12 match Autos run. It is not on the Driver Station list by
 * itself. Each of the 12 small files in this folder (for example AutoBlueD6ThreeVolley) just picks:
 *   - ALLIANCE: BLUE or RED,
 *   - SPOT    : where the robot starts (see "THE 4 START SPOTS"),
 *   - ROUTINE : MOVE_PARK, SHOOT_PARK or THREE_VOLLEY (see "THE 3 ROUTINES").
 * Fix a bug HERE and all 12 Autos get the fix. (One small class that "extends" this one, no deeper:
 * 12 copies of 1000 lines would never stay the same.)
 *
 * REAL COMPETITION FIELD, BLUE NUMBERS ONLY
 *   Every spot below is written ONCE, for BLUE. RED is the same spot turned 180 degrees around the
 *   middle of the field (red X = 141.5 - blue X, red Y = 141.5 - blue Y, red heading = blue + 180),
 *   the same rule as pedro/FieldConstants.java. NOT a mirror: the flowers and loading zones are
 *   only the same after a 180 degree turn. Pedro's map: origin at the bottom-left corner (audience
 *   at the bottom, RED area on the left, BLUE on the right), heading 0 faces right, 90 faces up.
 *
 * THE 4 START SPOTS (all touching the wall, facing the hive, in the tile column in front of it)
 *   MAIN  : the start the other Autos use (FieldConstants). BLUE: tile D6, rear wall, facing down.
 *           RED: tile C1, audience wall, facing up.
 *   FLOWER: straight across the field from MAIN, in the same column, right next to our flower.
 *           BLUE: tile D1, audience wall, facing up (about half an inch from the audience-wall
 *           flower). RED: tile C6, rear wall, facing down (next to the rear-wall flower).
 *           Starting that close to a flower, every path from here first drives STRAIGHT off the
 *           wall before it moves sideways or turns, so it never gets closer to the flower.
 *
 * THE 3 ROUTINES
 *   MOVE_PARK    : drive to our LOADING ZONE (blue F2, red A5) and stop. LEAVE + PARK.
 *   SHOOT_PARK   : shoot the 4 preloaded POLLEN from the start, then MOVE_PARK.
 *   THREE_VOLLEY : (BLUE numbers, red is turned 180)
 *     1. SHOT 1               : from the start spot.
 *     2. DRIVE_TO_FLOWER_A    : curve around the right side of the hive to the AUDIENCE-wall flower
 *                               (tile D|E 1), facing the audience wall. The bar drops on the way in.
 *     3. FLOWER A             : bar down, creep FLOWER_A_CREEP_INCHES in, pick.
 *     4. DRIVE_TO_SHOT_2      : back straight up SHOT_2_BACK_UP_INCHES, no turn.
 *     5. SHOT 2               : the hive is behind the robot. The turret swings LEFT and stops at
 *                               its +180 limit, about 11 degrees short of the hive. It can't turn
 *                               farther, so the camera never locks: after LOCK_TIMEOUT_SECONDS it
 *                               fires from there, 11 degrees off. NOT checked on the robot yet.
 *     6. DRIVE_TO_FLOWER_B    : curve up to the BLUE-wall flower (right wall, row 4|5), turning to
 *                               face the right wall.
 *     7. FLOWER B             : the bar drops only once the robot has stopped (it hit the flower when
 *                               it dropped early), creep FLOWER_B_CREEP_INCHES in, pick.
 *     8. DRIVE_TO_SHOT_3      : back off the flower straight, then turn and drive to the shot 3
 *                               spot: the start spot (MAIN), or SHOT_3 (FLOWER spot: the start is
 *                               jammed against the flower, so it shoots from a little way out).
 *     9. SHOT 3
 *    10. DRIVE_TO_PARK        : to the loading zone.
 *   From the FLOWER spot, flower A is right next to the start: the robot drives straight up off the
 *   wall, curves over and turns around (to face the audience wall) on the way down to flower A.
 *   The THREE_VOLLEY numbers (stops, creeps, curves, bar timing) are the ones tuned on the practice
 *   field with ThreeVolleyMirrorAlpha (2026-11-17), moved onto the real flowers. That plan, turned
 *   so it lands on the real blue flowers, is exactly this plan from the MAIN spot.
 *
 * INIT BUTTONS (gamepad 1). The screen shows what is picked.
 *   B = NONSTOP (the default): runs like a match.   A = STEP: stops after each drive, press A.
 *   Y = ACTIONS ON (the default): real shooting and flower picking.
 *   X = ACTIONS OFF: DRIVE ONLY, every shot and pick is a short pause.
 *   With no buttons pressed it runs as a real match Auto: NONSTOP, ACTIONS ON. (In a match nobody
 *   can press A, so a STEP-mode Auto would stop after its first drive.)
 *
 * ONE SHOT (the same sequence as ShootAndParkAuto)
 *   PRE-AIM        : the turret turns to where the hive SHOULD be (from the pose and the hive spot).
 *   FIND_HIVE      : the turret turns until the camera is locked. The shot table gives the RPM. If
 *                    the camera never locks, it shoots anyway at PRE_REV_FLYWHEEL_RPM from the
 *                    pre-aim angle (the run log says "locked at -1.0 s").
 *   WAIT_FOR_SPEED : the flywheel gets to that RPM and holds it, and the turret is on target.
 *   FEED_BALLS     : gate OPENED, the intake feeds, only while the flywheel is at speed. The flower
 *                    bar goes back UP here if it was still down from a flower.
 *   LIFT_LAST_BALL : gate LIFTED to push the last ball in, then gate CLOSED.
 *   Between shots the flywheel keeps spinning at PRE_REV_FLYWHEEL_RPM. It stops after the last one.
 *
 * ONE FLOWER (THREE_VOLLEY)
 *   On the way in, FLOWER_x_BAR_DOWN_INCHES before the stop, the bar goes DOWN and the intake starts.
 *   At the stop: wait until the bar is down, creep FLOWER_x_CREEP_INCHES straight in, sit there with
 *   the intake on for FLOWER_PICK_SECONDS, then stop the intake and drive on with the bar STILL DOWN.
 *   It goes back UP when the next shot starts feeding (or when that shot ends or is skipped).
 *
 * THE 30 SECOND LIMIT: once the robot has run SKIP_ACTIONS_AFTER_SECONDS (not counting time
 * waiting for A in step mode), shots and flowers are skipped or cut short so the robot still parks.
 * THREE_VOLLEY is long: if it keeps running out of time, shot 3 is the first thing to go.
 *
 * Pedro's Follower owns the drive motors and the Pinpoint, so this does NOT use DriveSubsystem or
 * PinpointSubsystem. Paths are checked (by a geometry script, not on the robot) to stay at least
 * 3.5 inches from the hive, the flowers and the walls, except where the plan touches a flower on
 * purpose. CHECK EACH AUTO ON THE FIELD (STEP mode first) before a match.
 */
public class BiobuzzAuto extends OpMode {

    // ==================================================================
    // What the small Auto files pick
    // ==================================================================
    public enum StartSpot { MAIN, FLOWER }

    public enum Routine { MOVE_PARK, SHOOT_PARK, THREE_VOLLEY }

    private final VisionSubsystem.Alliance alliance;
    private final StartSpot startSpot;
    private final Routine routine;

    public BiobuzzAuto(VisionSubsystem.Alliance alliance, StartSpot startSpot, Routine routine) {
        this.alliance = alliance;
        this.startSpot = startSpot;
        this.routine = routine;
    }

    // ==================================================================
    // Settings (change these to change every Auto)
    // ==================================================================
    // ACTIONS OFF: how long each shot and pick pause lasts (seconds).
    private static final double SHOOT_PAUSE_SECONDS = 3.0;
    private static final double INTAKE_PAUSE_SECONDS = 1.0;

    // --- Shooting (the same numbers as ShootAndParkAuto) ---
    // How long the gate stays OPENED while the intake feeds balls, in seconds.
    private static final double FEED_SECONDS = 3.0;
    // How long the gate stays LIFTED (with the intake still running) to push the last ball in.
    private static final double GATE_LIFT_SECONDS = 0.5;
    // true = the intake runs only while the flywheel is at speed (balls go one at a time).
    private static final boolean INTAKE_WAITS_FOR_SPEED = true;
    // The flywheel speed while looking for the hive, between shots, and for a shot without the
    // camera. The shot table's RPM takes over once the camera is locked. 1458 or lower.
    private static final double PRE_REV_FLYWHEEL_RPM = 1300.0;
    private static final double LOCK_TIMEOUT_SECONDS = 3.0;
    private static final double SPIN_UP_TIMEOUT_SECONDS = 6.0;
    private static final double FLYWHEEL_AT_SPEED_TOLERANCE_RPM = 25.0;
    private static final double FLYWHEEL_AT_SPEED_HOLD_SECONDS = 0.15;
    private static final double ALIGN_REAIM_THRESHOLD_DEGREES = 1.0;
    private static final double LOCKED_TOLERANCE_DEGREES = 2.0;

    // --- Flowers (THREE_VOLLEY) ---
    // true = lower the flower bar at each flower, and hold it UP the rest of the time.
    // false = never touch the bar (flowers then only run the intake). If the robot stops following
    // its curves, the bar's UP number is wrong and it is dragging: set this to false.
    private static final boolean USE_FLOWER_BAR = true;
    // How long the robot sits at a flower with the intake on, after the creep (seconds).
    private static final double FLOWER_PICK_SECONDS = 2.5;
    // How far before each flower stop the bar goes DOWN and the intake starts (inches).
    // Flower B uses 0: its bar hit the flower when it dropped early, so it drops once stopped.
    private static final double FLOWER_A_BAR_DOWN_INCHES = 20.0;
    private static final double FLOWER_B_BAR_DOWN_INCHES = 0.0;
    // How far the robot creeps straight into each flower once the bar is down (inches).
    private static final double FLOWER_A_CREEP_INCHES = 5.0;
    private static final double FLOWER_B_CREEP_INCHES = 8.0;
    // The most time a creep may take before the pick starts anyway (it may stop against the flower).
    private static final double CREEP_TIMEOUT_SECONDS = 1.5;

    // --- Time ---
    // Once the robot has run this long, shots and flowers are skipped or cut short so it can park.
    private static final double SKIP_ACTIONS_AFTER_SECONDS = 24.0;
    // The most time one drive may take.
    private static final double LEG_TIMEOUT_SECONDS = 8.0;

    // What the drive does at each stop: true = Pedro HOLDS the spot, false = the motors stop.
    // false because holding made the robot shake on 2026-11-17 (see ThreeVolleyMirrorAlpha).
    private static final boolean HOLD_AT_STOPS = false;

    // ==================================================================
    // Field spots: BLUE numbers only (inches, degrees). Red is built by turning these 180 degrees.
    // ==================================================================
    // START, MAIN spot: tile D6, touching the rear wall, facing down at the hive (FieldConstants).
    private static final double START_MAIN_X_INCHES = FieldConstants.BLUE_START_X_INCHES;
    private static final double START_MAIN_Y_INCHES = FieldConstants.BLUE_START_Y_INCHES;
    private static final double START_MAIN_HEADING_DEGREES = FieldConstants.BLUE_HEADING_DEGREES;
    // START, FLOWER spot: tile D1, the same column, touching the audience wall, facing up at the
    // hive. The robot's right side is about half an inch from the audience-wall flower.
    private static final double START_FLOWER_X_INCHES = FieldConstants.BLUE_START_X_INCHES;
    private static final double START_FLOWER_Y_INCHES = FieldConstants.ROBOT_LENGTH_INCHES / 2.0;
    private static final double START_FLOWER_HEADING_DEGREES = 90.0;
    // PARK: our loading zone, from FieldConstants (blue F2, red A5). From the FLOWER spot the robot
    // parks facing up instead of down (it never turns around on the way), same spot.

    // FLOWER A: the audience-wall flower (its center is at X 94.3). Stop facing the audience wall,
    // then creep FLOWER_A_CREEP_INCHES in.
    private static final double FLOWER_A_X_INCHES = 94.3;
    private static final double FLOWER_A_Y_INCHES = 18.0;
    private static final double FLOWER_A_HEADING_DEGREES = -90.0;
    // SHOT 2: this many inches straight back from where the flower A creep ends, no turn.
    private static final double SHOT_2_BACK_UP_INCHES = 7.0;

    // FLOWER B: the blue-wall flower (right wall, its center is at Y 94.3). Stop facing the wall,
    // then creep FLOWER_B_CREEP_INCHES in.
    private static final double FLOWER_B_X_INCHES = 123.6;
    private static final double FLOWER_B_Y_INCHES = 94.3;
    private static final double FLOWER_B_HEADING_DEGREES = 0.0;
    // Leaving flower B: how much of the drive back to the start (0 to 1) is straight back off the
    // flower before the robot starts turning, so its corners do not scrape the flower.
    private static final double LEAVE_FLOWER_B_BEFORE_TURN_FRACTION = 0.15;

    // The BLUE hive, for the PRE-AIM only (derived from the manual, not measured).
    private static final double HIVE_X_INCHES = 83.5;
    private static final double HIVE_Y_INCHES = 70.75;

    // Curve control points (inches). A control point bends the path toward it. The robot does NOT
    // drive through it.
    // Start -> flower A: swings right, around the hive.
    private static final double TO_FLOWER_A_CONTROL1_X = 126.0;
    private static final double TO_FLOWER_A_CONTROL1_Y = 126.5;
    private static final double TO_FLOWER_A_CONTROL2_X = 126.0;
    private static final double TO_FLOWER_A_CONTROL2_Y = 31.5;
    // Shot 2 -> flower B.
    private static final double TO_FLOWER_B_CONTROL_X = 122.0;
    private static final double TO_FLOWER_B_CONTROL_Y = 36.5;
    // MAIN start -> park: around the right side of the hive.
    private static final double MAIN_TO_PARK_CONTROL1_X = 122.0;
    private static final double MAIN_TO_PARK_CONTROL1_Y = 120.0;
    private static final double MAIN_TO_PARK_CONTROL2_X = 124.0;
    private static final double MAIN_TO_PARK_CONTROL2_Y = 70.0;

    // FLOWER spot paths. The first control point is straight above the start (a little to the left)
    // so the robot drives straight off the wall, away from the flower, before it moves right.
    // FLOWER start -> park.
    private static final double FLOWER_TO_PARK_CONTROL1_X = 78.0;
    private static final double FLOWER_TO_PARK_CONTROL1_Y = 30.0;
    private static final double FLOWER_TO_PARK_CONTROL2_X = 105.0;
    private static final double FLOWER_TO_PARK_CONTROL2_Y = 30.0;
    // FLOWER start -> flower A: up off the wall, over, and back down onto the flower, turning
    // around (90 -> 0 -> -90, clockwise) on the way. The turn waits until the robot is clear of the
    // wall (TURN_START_FRACTION) and is half done at TURN_MIDDLE_FRACTION.
    private static final double FLOWER_TO_FLOWER_A_CONTROL1_X = 78.0;
    private static final double FLOWER_TO_FLOWER_A_CONTROL1_Y = 36.0;
    private static final double FLOWER_TO_FLOWER_A_CONTROL2_X = 94.3;
    private static final double FLOWER_TO_FLOWER_A_CONTROL2_Y = 42.0;
    private static final double FLOWER_TO_FLOWER_A_TURN_START_FRACTION = 0.35;
    private static final double FLOWER_TO_FLOWER_A_TURN_MIDDLE_FRACTION = 0.70;
    // FLOWER spot, SHOT 3: a little way out from the start, facing up at the hive (the hive is
    // about 26 degrees to the turret's right from here).
    private static final double FLOWER_SHOT_3_X_INCHES = 104.0;
    private static final double FLOWER_SHOT_3_Y_INCHES = 28.0;
    private static final double FLOWER_SHOT_3_HEADING_DEGREES = 90.0;

    // ==================================================================
    // The steps. The robot is always in exactly one of these.
    // ==================================================================
    // true once START was pressed. stop() only saves the ending heading for the TeleOp if the Auto
    // really ran (an Auto that was only INIT'd and then stopped must not hand anything over).
    private boolean autoStarted = false;

    private enum AutoState {
        SHOT_PAUSE, SHOT_FIND_HIVE, SHOT_WAIT_FOR_SPEED, SHOT_FEED_BALLS, SHOT_LIFT_LAST_BALL,
        FLOWER_BAR_DOWN, FLOWER_CREEP, FLOWER_PICK, FLOWER_BAR_UP,
        DRIVE_TO_FLOWER_A, DRIVE_TO_SHOT_2, DRIVE_TO_FLOWER_B, DRIVE_TO_SHOT_3, DRIVE_TO_PARK,
        WAIT_FOR_BUTTON,   // STEP mode only: stopped after a drive, waiting for A
        DONE
    }

    // ==================================================================
    // Things this OpMode uses
    // ==================================================================
    private Follower follower;
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;
    private FlowerPickerSubsystem flowerPickerSubsystem;   // null when USE_FLOWER_BAR is false
    private ShotTable shotTable;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses, already turned for red.
    private Pose startPose;
    private Pose parkPose;
    private Pose flowerAPose;
    private Pose flowerACreepPose;
    private Pose shot2Pose;
    private Pose flowerBPose;
    private Pose flowerBCreepPose;
    private Pose shot3Pose;   // MAIN: the start. FLOWER: FLOWER_SHOT_3.
    private Pose hivePose;

    // Paths, built once in init().
    private Path startToParkPath;
    private Path toFlowerAPath;
    private Path flowerACreepPath;
    private Path toShot2Path;
    private Path toFlowerBPath;
    private Path flowerBCreepPath;
    private Path flowerBToShot3Path;
    private Path shot3ToParkPath;

    private int lastShotNumber;   // 0, 1 or 3, from the routine

    // Picked in INIT.
    private boolean stepMode = false;   // A = true, B = false
    private boolean actionsOn = true;   // Y = true, X = false

    private AutoState autoState = AutoState.DONE;
    private int shotNumber = 0;
    private char flower = 'A';          // 'A' or 'B'
    private boolean barIsDownForFlower = false;

    private AutoState driveThatFinished = AutoState.DONE;
    private boolean aWasPressed = false;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime legTimer = new ElapsedTime();
    private final ElapsedTime runTimer = new ElapsedTime();
    private final ElapsedTime buttonWaitTimer = new ElapsedTime();
    private double totalButtonWaitSeconds = 0.0;
    private double finishedRunSeconds = 0.0;

    private double shotFlywheelRpm = 0.0;
    private boolean flywheelReady = false;
    private boolean flywheelWasInTolerance = false;
    private final ElapsedTime flywheelHoldTimer = new ElapsedTime();
    private boolean feedWasReady = false;
    private int flywheelDipCount = 0;
    private double lockedAtSeconds = 0.0;

    // The run log. It stays on the screen after the run.
    private final String[] shotResults = {"-", "-", "-"};
    private String flowerAResult = "-";
    private String flowerBResult = "-";
    private String legTimeouts = "none";
    private String lastStopName = "none yet";
    private double lastStopErrorInches = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Poses (BLUE numbers, turned for red by makePose).
        if (startSpot == StartSpot.MAIN) {
            startPose = makePose(START_MAIN_X_INCHES, START_MAIN_Y_INCHES, START_MAIN_HEADING_DEGREES);
            parkPose = FieldConstants.parkPose(alliance);
        } else {
            startPose = makePose(START_FLOWER_X_INCHES, START_FLOWER_Y_INCHES, START_FLOWER_HEADING_DEGREES);
            // Same park spot, but facing the way the robot started (up, for blue).
            parkPose = makePose(FieldConstants.BLUE_PARK_X_INCHES, FieldConstants.BLUE_PARK_Y_INCHES,
                    START_FLOWER_HEADING_DEGREES);
        }
        flowerAPose = makePose(FLOWER_A_X_INCHES, FLOWER_A_Y_INCHES, FLOWER_A_HEADING_DEGREES);
        flowerACreepPose = poseAhead(flowerAPose, FLOWER_A_CREEP_INCHES);
        shot2Pose = poseAhead(flowerACreepPose, -SHOT_2_BACK_UP_INCHES);
        flowerBPose = makePose(FLOWER_B_X_INCHES, FLOWER_B_Y_INCHES, FLOWER_B_HEADING_DEGREES);
        flowerBCreepPose = poseAhead(flowerBPose, FLOWER_B_CREEP_INCHES);
        shot3Pose = startSpot == StartSpot.MAIN ? startPose
                : makePose(FLOWER_SHOT_3_X_INCHES, FLOWER_SHOT_3_Y_INCHES, FLOWER_SHOT_3_HEADING_DEGREES);
        hivePose = makePose(HIVE_X_INCHES, HIVE_Y_INCHES, 0.0);

        if (routine == Routine.MOVE_PARK) {
            lastShotNumber = 0;
        } else if (routine == Routine.SHOOT_PARK) {
            lastShotNumber = 1;
        } else {
            lastShotNumber = 3;
        }
        for (int i = 0; i < lastShotNumber; i++) {
            shotResults[i] = "not yet";
        }
        if (routine == Routine.THREE_VOLLEY) {
            flowerAResult = "not yet";
            flowerBResult = "not yet";
        }

        // Create the Follower. This recalibrates the Pinpoint, so keep the robot still.
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        // The subsystems are always created (so ACTIONS can change during INIT). With ACTIONS OFF
        // they are never told to do anything.
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);   // gate moves to CLOSED
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        turretSubsystem = new TurretSubsystem(hardwareMap);               // does not move at INIT
        visionSubsystem = new VisionSubsystem(hardwareMap);
        visionSubsystem.setAlliance(alliance);
        shotTable = new ShotTable();
        if (USE_FLOWER_BAR) {
            // The bar moves UP here. (Rule R103.B allows an Auto to set servos during INIT.)
            flowerPickerSubsystem = new FlowerPickerSubsystem(hardwareMap);
        }

        buildPaths();
        telemetry.addLine("Pinpoint ready. Keep the robot still.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (gamepad1.a) {
            stepMode = true;
        } else if (gamepad1.b) {
            stepMode = false;
        }
        if (gamepad1.y) {
            actionsOn = true;
        } else if (gamepad1.x) {
            actionsOn = false;
        }

        follower.update();
        visionSubsystem.update();
        turretSubsystem.update();   // read only, nothing moves during INIT

        addAutoNameTelemetry();
        addSafetyTelemetry();
        telemetry.addData("Mode (A = step, B = nonstop)", stepMode ? "STEP" : "NONSTOP");
        telemetry.addData("Actions (Y = on, X = off)", actionsOn ? "ON" : "OFF: drive only");
        if (actionsOn && routine != Routine.MOVE_PARK) {
            addInitCameraTelemetry();
        }
        addPoseTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        follower.setPose(startPose);
        follower.update();

        // Tell the TeleOp which alliance this Auto played, so it starts on the same one.
        AutoHandoff.save(alliance);
        autoStarted = true;
        aWasPressed = gamepad1.a;
        runTimer.reset();
        totalButtonWaitSeconds = 0.0;
        flywheelReady = false;
        flywheelWasInTolerance = false;

        if (routine == Routine.MOVE_PARK) {
            startDrive(startToParkPath, AutoState.DRIVE_TO_PARK);
            return;
        }
        if (actionsOn) {
            shooterSubsystem.setTargetRpm(PRE_REV_FLYWHEEL_RPM);
        }
        startShot(1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors. Pedro must update every loop.
        follower.update();
        visionSubsystem.update();

        boolean aNewPress = gamepad1.a && !aWasPressed;
        aWasPressed = gamepad1.a;

        // Keep the flower bar UP, except while it is down for a flower.
        if (USE_FLOWER_BAR && autoState != AutoState.FLOWER_PICK && !barIsDownForFlower) {
            flowerPickerSubsystem.goToUp();
        }

        if (actionsOn && isOutOfTime()) {
            cutShortIfBusy();
        }

        // 2. Decide what to do next.
        switch (autoState) {
            // ---------------- ONE SHOT ----------------
            case SHOT_PAUSE:
                if (stateTimer.seconds() >= SHOOT_PAUSE_SECONDS) {
                    shotResults[shotNumber - 1] = "pause (actions off)";
                    finishShot();
                }
                break;

            case SHOT_FIND_HIVE:
                shooterSubsystem.setTargetRpm(PRE_REV_FLYWHEEL_RPM);
                aimTurretAtHive();
                if (isHiveLocked()) {
                    lockedAtSeconds = robotSeconds();
                    shotFlywheelRpm = shotTable.getFlywheelRpm(visionSubsystem.getTyDegrees());
                    shooterSubsystem.setTargetRpm(shotFlywheelRpm);
                    stateTimer.reset();
                    autoState = AutoState.SHOT_WAIT_FOR_SPEED;
                } else if (stateTimer.seconds() >= LOCK_TIMEOUT_SECONDS) {
                    // No camera lock: shoot anyway at the pre-rev speed, from the pre-aim angle.
                    lockedAtSeconds = -1.0;
                    shotFlywheelRpm = PRE_REV_FLYWHEEL_RPM;
                    stateTimer.reset();
                    autoState = AutoState.SHOT_WAIT_FOR_SPEED;
                }
                break;

            case SHOT_WAIT_FOR_SPEED:
                shooterSubsystem.setTargetRpm(shotFlywheelRpm);
                aimTurretAtHive();
                // A shot without the camera (lockedAtSeconds < 0) does not wait for the turret.
                if (flywheelReady && (turretSubsystem.isOnTarget() || lockedAtSeconds < 0)) {
                    barIsDownForFlower = false;   // the feed starts: the flower bar can go UP
                    shooterServoSubsystem.goToOpened();
                    feedWasReady = true;
                    flywheelDipCount = 0;
                    stateTimer.reset();
                    autoState = AutoState.SHOT_FEED_BALLS;
                } else if (stateTimer.seconds() >= SPIN_UP_TIMEOUT_SECONDS) {
                    skipShot("SKIPPED: the flywheel never reached speed");
                }
                break;

            case SHOT_FEED_BALLS:
                if (flywheelReady || !INTAKE_WAITS_FOR_SPEED) {
                    intakeSubsystem.intake();
                } else {
                    intakeSubsystem.stop();
                }
                if (feedWasReady && !flywheelReady) {
                    flywheelDipCount = flywheelDipCount + 1;   // about one per ball
                }
                feedWasReady = flywheelReady;
                if (stateTimer.seconds() >= FEED_SECONDS) {
                    shooterServoSubsystem.goToLifted();
                    stateTimer.reset();
                    autoState = AutoState.SHOT_LIFT_LAST_BALL;
                }
                break;

            case SHOT_LIFT_LAST_BALL:
                intakeSubsystem.intake();
                if (stateTimer.seconds() >= GATE_LIFT_SECONDS) {
                    shotResults[shotNumber - 1] = String.format(
                            "SHOT (locked at %.1f s, %.0f RPM, %d dips)",
                            lockedAtSeconds, shotFlywheelRpm, flywheelDipCount);
                    finishShot();
                }
                break;

            // ---------------- ONE FLOWER ----------------
            case FLOWER_BAR_DOWN:
                // ACTIONS ON: bar DOWN and the intake on. Wait until the bar is all the way down.
                if (USE_FLOWER_BAR) {
                    flowerPickerSubsystem.goToDown();
                }
                intakeSubsystem.intake();
                if (!USE_FLOWER_BAR || flowerPickerSubsystem.isAtPosition()) {
                    startCreep();
                }
                break;

            case FLOWER_CREEP:
                if (actionsOn) {
                    if (USE_FLOWER_BAR) {
                        flowerPickerSubsystem.goToDown();
                    }
                    intakeSubsystem.intake();
                }
                if (!follower.isBusy() || legTimer.seconds() >= CREEP_TIMEOUT_SECONDS) {
                    if (!HOLD_AT_STOPS) {
                        follower.stop();
                    }
                    stateTimer.reset();
                    autoState = AutoState.FLOWER_PICK;
                }
                break;

            case FLOWER_PICK:
                if (actionsOn) {
                    if (USE_FLOWER_BAR) {
                        flowerPickerSubsystem.goToDown();
                    }
                    intakeSubsystem.intake();
                }
                double pickSeconds = actionsOn ? FLOWER_PICK_SECONDS : INTAKE_PAUSE_SECONDS;
                if (stateTimer.seconds() >= pickSeconds) {
                    setFlowerResult(actionsOn
                            ? String.format("picked for %.1f s", FLOWER_PICK_SECONDS)
                            : "pause (actions off)");
                    // Keep the bar DOWN. It goes UP when the next shot starts feeding.
                    finishFlower();
                }
                break;

            case FLOWER_BAR_UP:
                // Only used when a flower is cut short. The goToUp() at the top of loop() sends it.
                intakeSubsystem.intake();
                if (!USE_FLOWER_BAR || flowerPickerSubsystem.isAtPosition()) {
                    finishFlower();
                }
                break;

            // ---------------- DRIVES ----------------
            case DRIVE_TO_FLOWER_A:
                lowerBarNearFlower(FLOWER_A_BAR_DOWN_INCHES);
                if (isDriveDone("flower A")) {
                    finishDrive("Flower A", flowerAPose, autoState);
                }
                break;

            case DRIVE_TO_SHOT_2:
                if (isDriveDone("shot 2")) {
                    finishDrive("Shot 2", shot2Pose, autoState);
                }
                break;

            case DRIVE_TO_FLOWER_B:
                lowerBarNearFlower(FLOWER_B_BAR_DOWN_INCHES);
                if (isDriveDone("flower B")) {
                    finishDrive("Flower B", flowerBPose, autoState);
                }
                break;

            case DRIVE_TO_SHOT_3:
                if (isDriveDone("shot 3")) {
                    finishDrive("Shot 3", shot3Pose, autoState);
                }
                break;

            case DRIVE_TO_PARK:
                if (isDriveDone("park")) {
                    finishedRunSeconds = robotSeconds();
                    finishDrive("Park", parkPose, autoState);
                }
                break;

            case WAIT_FOR_BUTTON:
                if (aNewPress) {
                    totalButtonWaitSeconds = totalButtonWaitSeconds + buttonWaitTimer.seconds();
                    continueAfterDrive(driveThatFinished);
                }
                break;

            case DONE:
                break;
        }

        // The subsystems need update() once per loop, AFTER they get their commands.
        shooterSubsystem.update();
        turretSubsystem.update();
        intakeSubsystem.update();
        updateFlywheelReady();

        // 3. Telemetry
        addAutoNameTelemetry();
        addSafetyTelemetry();
        telemetry.addData("Mode / Actions", "%s / %s",
                stepMode ? "STEP" : "NONSTOP", actionsOn ? "ON" : "OFF (drive only)");
        telemetry.addData("State", autoState);
        if (autoState == AutoState.WAIT_FOR_BUTTON) {
            telemetry.addLine(">>> Measure the robot against the tape, then press A <<<");
        }
        if (autoState == AutoState.DONE && finishedRunSeconds > 0) {
            telemetry.addData("Run time (s, button waits removed)", "%.1f", finishedRunSeconds);
        } else {
            telemetry.addData("Run time so far (s)", "%.1f", robotSeconds());
        }
        if (actionsOn && isOutOfTime()) {
            telemetry.addLine("OUT OF TIME: skipping shots and flowers, going to park");
        }
        telemetry.addData("Last stop / error (in)", "%s / %.2f", lastStopName, lastStopErrorInches);
        if (actionsOn && routine != Routine.MOVE_PARK) {
            addShotTelemetry();
        }
        addRunLogTelemetry();
        addPoseTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Hand the robot's ending heading to the TeleOp (only if this Auto really ran), so the
        // field-centric drive knows which way the robot faces. Pedro's heading is in radians.
        if (autoStarted) {
            AutoHandoff.saveHeading(Math.toDegrees(follower.pose().heading()));
        }
        follower.stop();
        follower.drivetrain.stop();
        shooterSubsystem.stop();
        intakeSubsystem.stop();
        turretSubsystem.stop();   // floats; the hub keeps its encoder count for TeleOp
        visionSubsystem.stop();
    }

    // ==================================================================
    // The routine: what comes next
    // ==================================================================

    // After shot 1, 2 or 3.
    private void afterShot() {
        if (shotNumber >= lastShotNumber) {
            // Park from where the last shot was: the start (shot 1), or the shot 3 spot.
            startDrive(shotNumber == 3 ? shot3ToParkPath : startToParkPath, AutoState.DRIVE_TO_PARK);
        } else if (shotNumber == 1) {
            startDrive(toFlowerAPath, AutoState.DRIVE_TO_FLOWER_A);
        } else {
            startDrive(toFlowerBPath, AutoState.DRIVE_TO_FLOWER_B);
        }
    }

    // After flower A or B.
    private void afterFlower() {
        if (flower == 'A') {
            startDrive(toShot2Path, AutoState.DRIVE_TO_SHOT_2);
        } else {
            startDrive(flowerBToShot3Path, AutoState.DRIVE_TO_SHOT_3);
        }
    }

    // After each drive (right away, or after A in STEP mode).
    private void continueAfterDrive(AutoState driveState) {
        switch (driveState) {
            case DRIVE_TO_FLOWER_A:
                startFlower('A');
                break;
            case DRIVE_TO_SHOT_2:
                startShot(2);
                break;
            case DRIVE_TO_FLOWER_B:
                startFlower('B');
                break;
            case DRIVE_TO_SHOT_3:
                startShot(3);
                break;
            default:
                autoState = AutoState.DONE;
                break;
        }
    }

    // ==================================================================
    // Shots
    // ==================================================================
    private void startShot(int number) {
        shotNumber = number;
        stateTimer.reset();
        if (!actionsOn) {
            autoState = AutoState.SHOT_PAUSE;
            return;
        }
        if (isOutOfTime()) {
            skipShot("SKIPPED: out of time");
            return;
        }
        shooterServoSubsystem.goToClosed();
        turretSubsystem.setTargetAngle(turretAngleToHiveDegrees());   // PRE-AIM
        autoState = AutoState.SHOT_FIND_HIVE;
    }

    private void skipShot(String reason) {
        shotResults[shotNumber - 1] = reason;
        finishShot();
    }

    private void finishShot() {
        barIsDownForFlower = false;
        if (actionsOn) {
            shooterServoSubsystem.goToClosed();
            intakeSubsystem.stop();
            turretSubsystem.holdCurrentAngle();
            if (shotNumber < lastShotNumber) {
                shooterSubsystem.setTargetRpm(PRE_REV_FLYWHEEL_RPM);   // keep it spinning
            } else {
                shooterSubsystem.stop();                               // last shot
            }
        }
        afterShot();
    }

    // The turret angle that points at the hive from where the robot is now (0 straight ahead,
    // positive to the LEFT). It is always an angle the turret can actually reach (see below).
    private double turretAngleToHiveDegrees() {
        Pose pose = follower.pose();
        double toHiveDegrees = Math.toDegrees(
                Math.atan2(hivePose.y() - pose.y(), hivePose.x() - pose.x()));
        double wantedDegrees = wrapDegrees(toHiveDegrees - Math.toDegrees(pose.heading()));
        return closestReachableTurretAngle(wantedDegrees);
    }

    // The turret can't spin all the way around. Its soft limits (in TurretSubsystem) are not the
    // same on both sides: it turns farther LEFT than RIGHT. One direction can be written three ways:
    // wanted, wanted + 360 (the same direction, turning left the long way) and wanted - 360 (turning
    // right the long way). Example, blue shot 2: the hive is at -169 (right). That is past the right
    // limit, but +191 (left) is the same direction and is only 11 degrees past the left limit.
    //   1. If one of the three is inside the limits, use it.
    //   2. If none is, stop AT the limit that gets the turret closest to the hive. The turret never
    //      goes past a limit, so the camera can't finish the aim from there: the shot is fired from
    //      the limit (after LOCK_TIMEOUT_SECONDS) and is off by however far the hive is past it.
    // The limits come from TurretSubsystem, so if they change there, this follows.
    private double closestReachableTurretAngle(double wantedDegrees) {
        double minDegrees = turretSubsystem.getMinAngleDegrees();
        double maxDegrees = turretSubsystem.getMaxAngleDegrees();
        double[] sameDirection = {wantedDegrees, wantedDegrees + 360.0, wantedDegrees - 360.0};

        // 1. Is one of them inside the limits?
        for (int i = 0; i < sameDirection.length; i++) {
            if (sameDirection[i] >= minDegrees && sameDirection[i] <= maxDegrees) {
                return sameDirection[i];
            }
        }

        // 2. None is. Clip each one to the limits and keep the one that misses by the least.
        double bestDegrees = Math.max(minDegrees, Math.min(maxDegrees, wantedDegrees));
        double bestMissDegrees = Math.abs(bestDegrees - wantedDegrees);
        for (int i = 1; i < sameDirection.length; i++) {
            double clippedDegrees = Math.max(minDegrees, Math.min(maxDegrees, sameDirection[i]));
            double missDegrees = Math.abs(clippedDegrees - sameDirection[i]);
            if (missDegrees < bestMissDegrees) {
                bestDegrees = clippedDegrees;
                bestMissDegrees = missDegrees;
            }
        }
        return bestDegrees;
    }

    // ==================================================================
    // Flowers
    // ==================================================================
    private void startFlower(char which) {
        flower = which;
        stateTimer.reset();
        if (actionsOn && isOutOfTime()) {
            barIsDownForFlower = false;
            setFlowerResult("SKIPPED: out of time");
            afterFlower();
            return;
        }
        if (!actionsOn) {
            startCreep();
            return;
        }
        shooterServoSubsystem.goToClosed();   // the balls wait at the gate
        barIsDownForFlower = true;
        autoState = AutoState.FLOWER_BAR_DOWN;
    }

    private void startCreep() {
        legTimer.reset();
        follower.follow(flower == 'A' ? flowerACreepPath : flowerBCreepPath);
        autoState = AutoState.FLOWER_CREEP;
    }

    // ACTIONS ON: while driving TO a flower, drop the bar and start the intake once the robot is
    // within barDownInches of the stop.
    private void lowerBarNearFlower(double barDownInches) {
        if (!actionsOn) {
            return;
        }
        if (!barIsDownForFlower && follower.distanceToEndpoint() <= barDownInches) {
            barIsDownForFlower = true;
            shooterServoSubsystem.goToClosed();
        }
        if (barIsDownForFlower) {
            if (USE_FLOWER_BAR) {
                flowerPickerSubsystem.goToDown();
            }
            intakeSubsystem.intake();
        }
    }

    private void finishFlower() {
        if (actionsOn) {
            intakeSubsystem.stop();
        }
        afterFlower();
    }

    private void setFlowerResult(String result) {
        if (flower == 'A') {
            flowerAResult = result;
        } else {
            flowerBResult = result;
        }
    }

    // ==================================================================
    // Running out of time (ACTIONS ON only)
    // ==================================================================
    private double robotSeconds() {
        double waitingNow = autoState == AutoState.WAIT_FOR_BUTTON ? buttonWaitTimer.seconds() : 0.0;
        return runTimer.seconds() - totalButtonWaitSeconds - waitingNow;
    }

    private boolean isOutOfTime() {
        return robotSeconds() >= SKIP_ACTIONS_AFTER_SECONDS;
    }

    private void cutShortIfBusy() {
        switch (autoState) {
            case SHOT_FIND_HIVE:
            case SHOT_WAIT_FOR_SPEED:
            case SHOT_FEED_BALLS:
            case SHOT_LIFT_LAST_BALL:
                skipShot("CUT SHORT: out of time");
                break;
            case FLOWER_BAR_DOWN:
            case FLOWER_CREEP:
            case FLOWER_PICK:
                setFlowerResult("CUT SHORT: out of time");
                if (!HOLD_AT_STOPS) {
                    follower.stop();
                }
                barIsDownForFlower = false;
                if (USE_FLOWER_BAR) {
                    flowerPickerSubsystem.goToUp();
                }
                autoState = AutoState.FLOWER_BAR_UP;
                break;
            default:
                break;
        }
    }

    // ==================================================================
    // Drives
    // ==================================================================
    private void startDrive(Path path, AutoState driveState) {
        if (!Constants.FORESIGHT_TUNED) {
            shooterSubsystem.stop();
            autoState = AutoState.DONE;
            return;
        }
        legTimer.reset();
        follower.follow(path);
        autoState = driveState;
    }

    private boolean isDriveDone(String driveName) {
        if (!follower.isBusy()) {
            return true;
        }
        if (legTimer.seconds() >= LEG_TIMEOUT_SECONDS) {
            legTimeouts = legTimeouts.equals("none") ? driveName : legTimeouts + ", " + driveName;
            return true;
        }
        return false;
    }

    private void finishDrive(String stopName, Pose targetPose, AutoState driveState) {
        if (!HOLD_AT_STOPS) {
            follower.stop();
        }
        Pose pose = follower.pose();
        lastStopName = stopName;
        lastStopErrorInches = Math.hypot(pose.x() - targetPose.x(), pose.y() - targetPose.y());

        if (stepMode && driveState != AutoState.DRIVE_TO_PARK) {
            driveThatFinished = driveState;
            buttonWaitTimer.reset();
            autoState = AutoState.WAIT_FOR_BUTTON;
        } else {
            continueAfterDrive(driveState);
        }
    }

    // ==================================================================
    // Aiming and the flywheel (same as ShootAndParkAuto)
    // ==================================================================
    private void aimTurretAtHive() {
        if (!visionSubsystem.hasTarget()) {
            return;
        }
        double txDegrees = visionSubsystem.getTxDegrees();
        boolean notAimingYet = turretSubsystem.getState() != TurretSubsystem.TurretState.POSITION;
        boolean farOff = Math.abs(txDegrees) > ALIGN_REAIM_THRESHOLD_DEGREES;
        if (notAimingYet || farOff) {
            // tx is positive to the RIGHT, turret angles are positive to the LEFT, so subtract.
            turretSubsystem.setTargetAngle(turretSubsystem.getAngleDegrees() - txDegrees);
        }
    }

    private boolean isHiveLocked() {
        return visionSubsystem.hasTarget()
                && Math.abs(visionSubsystem.getTxDegrees()) <= LOCKED_TOLERANCE_DEGREES
                && turretSubsystem.isOnTarget();
    }

    private void updateFlywheelReady() {
        double targetRpm = shooterSubsystem.getTargetFlywheelRpm();
        double errorRpm = Math.abs(shooterSubsystem.getFlywheelRpm() - targetRpm);
        boolean inTolerance = targetRpm > 0.0 && errorRpm <= FLYWHEEL_AT_SPEED_TOLERANCE_RPM;
        if (!inTolerance) {
            flywheelWasInTolerance = false;
            flywheelReady = false;
            return;
        }
        if (!flywheelWasInTolerance) {
            flywheelHoldTimer.reset();
            flywheelWasInTolerance = true;
        }
        flywheelReady = flywheelHoldTimer.seconds() >= FLYWHEEL_AT_SPEED_HOLD_SECONDS;
    }

    // ==================================================================
    // Poses and paths
    // ==================================================================

    // A pose from BLUE numbers. For RED it is turned 180 degrees around the field's middle.
    private Pose makePose(double blueX, double blueY, double blueHeadingDegrees) {
        double x = blueX;
        double y = blueY;
        double headingDegrees = blueHeadingDegrees;
        if (alliance == VisionSubsystem.Alliance.RED) {
            x = FieldConstants.FIELD_SIZE_INCHES - blueX;
            y = FieldConstants.FIELD_SIZE_INCHES - blueY;
            headingDegrees = wrapDegrees(blueHeadingDegrees + 180.0);
        }
        return poseFactory.of(x, y, headingDegrees);
    }

    // A control point from BLUE numbers (control points have no heading).
    private Pose makePoint(double blueX, double blueY) {
        return makePose(blueX, blueY, 0.0);
    }

    // The pose a number of inches straight ahead of a pose (negative = straight back).
    private Pose poseAhead(Pose pose, double inches) {
        return new Pose(pose.x() + inches * Math.cos(pose.heading()),
                pose.y() + inches * Math.sin(pose.heading()),
                pose.heading());
    }

    private void buildPaths() {
        if (startSpot == StartSpot.MAIN) {
            // MAIN start -> park: around the right side of the hive (blue numbers), still facing
            // down.
            startToParkPath = Paths.curve(startPose,
                    makePoint(MAIN_TO_PARK_CONTROL1_X, MAIN_TO_PARK_CONTROL1_Y),
                    makePoint(MAIN_TO_PARK_CONTROL2_X, MAIN_TO_PARK_CONTROL2_Y),
                    parkPose).constant(parkPose);
            // MAIN start -> flower A: curve around the hive, keep facing down.
            toFlowerAPath = Paths.curve(startPose,
                    makePoint(TO_FLOWER_A_CONTROL1_X, TO_FLOWER_A_CONTROL1_Y),
                    makePoint(TO_FLOWER_A_CONTROL2_X, TO_FLOWER_A_CONTROL2_Y),
                    flowerAPose).constant(flowerAPose);
        } else {
            // FLOWER start -> park: straight off the wall, then over to the zone, still facing up.
            startToParkPath = Paths.curve(startPose,
                    makePoint(FLOWER_TO_PARK_CONTROL1_X, FLOWER_TO_PARK_CONTROL1_Y),
                    makePoint(FLOWER_TO_PARK_CONTROL2_X, FLOWER_TO_PARK_CONTROL2_Y),
                    parkPose).constant(parkPose);
            // FLOWER start -> flower A: up off the wall facing up, then turn around clockwise
            // (90 -> 0 -> -90 for blue) while curving back down onto the flower.
            Pose sideways = makePose(0.0, 0.0, 0.0);   // only its heading is used: facing right
            Interpolator upThenTurnAround = Interpolator.piecewise()
                    .until(FLOWER_TO_FLOWER_A_TURN_START_FRACTION, Interpolator.constant(startPose))
                    .until(FLOWER_TO_FLOWER_A_TURN_MIDDLE_FRACTION, Interpolator.linear(startPose, sideways))
                    .until(1.0, Interpolator.linear(sideways, flowerAPose));
            toFlowerAPath = Paths.curve(startPose,
                    makePoint(FLOWER_TO_FLOWER_A_CONTROL1_X, FLOWER_TO_FLOWER_A_CONTROL1_Y),
                    makePoint(FLOWER_TO_FLOWER_A_CONTROL2_X, FLOWER_TO_FLOWER_A_CONTROL2_Y),
                    flowerAPose).heading(upThenTurnAround);
        }

        // THREE_VOLLEY paths (the same for both spots from flower A on).
        // Short straight creeps into each flower.
        flowerACreepPath = Paths.line(flowerAPose, flowerACreepPose).constant(flowerAPose);
        flowerBCreepPath = Paths.line(flowerBPose, flowerBCreepPose).constant(flowerBPose);
        // Flower A creep end -> shot 2: straight back, no turn.
        toShot2Path = Paths.line(flowerACreepPose, shot2Pose).constant(flowerACreepPose);
        // Shot 2 -> flower B: curve, turning to face the wall.
        toFlowerBPath = Paths.curve(shot2Pose,
                makePoint(TO_FLOWER_B_CONTROL_X, TO_FLOWER_B_CONTROL_Y),
                flowerBPose).linear(shot2Pose, flowerBPose);
        // Flower B creep end -> shot 3 spot: straight back off the flower first, then turn while
        // driving. A "piecewise" heading is one rule until a point on the path, then another rule.
        Interpolator backOffThenTurn = Interpolator.piecewise()
                .until(LEAVE_FLOWER_B_BEFORE_TURN_FRACTION, Interpolator.constant(flowerBCreepPose))
                .until(1.0, Interpolator.linear(flowerBCreepPose, shot3Pose));
        flowerBToShot3Path = Paths.line(flowerBCreepPose, shot3Pose).heading(backOffThenTurn);
        // Shot 3 -> park. MAIN: the same as start -> park. FLOWER: a short straight line.
        shot3ToParkPath = startSpot == StartSpot.MAIN ? startToParkPath
                : Paths.line(shot3Pose, parkPose).constant(parkPose);
    }

    private double wrapDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees = degrees - 360.0;
        }
        while (degrees < -180.0) {
            degrees = degrees + 360.0;
        }
        return degrees;
    }

    // ==================================================================
    // Telemetry
    // ==================================================================
    private void addAutoNameTelemetry() {
        String tile;
        if (alliance == VisionSubsystem.Alliance.BLUE) {
            tile = startSpot == StartSpot.MAIN ? "D6 (rear wall)" : "D1 (audience wall, by the flower)";
        } else {
            tile = startSpot == StartSpot.MAIN ? "C1 (audience wall)" : "C6 (rear wall, by the flower)";
        }
        telemetry.addData(">>> ALLIANCE", alliance);
        telemetry.addData(">>> START", "%s, facing the hive", tile);
        telemetry.addData(">>> ROUTINE", routine);
    }

    private void addSafetyTelemetry() {
        if (!Constants.FORESIGHT_TUNED) {
            telemetry.addLine("!!! NOT TUNED: this Auto will NOT drive. !!!");
        }
    }

    private void addInitCameraTelemetry() {
        telemetry.addData("Camera connected", visionSubsystem.isCameraConnected());
        if (visionSubsystem.hasTarget()) {
            double tyDegrees = visionSubsystem.getTyDegrees();
            telemetry.addData("Hive visible", "YES (ty %.1f)", tyDegrees);
            telemetry.addData("Table RPM from here", "%.0f  (PRE_REV_FLYWHEEL_RPM is %.0f)",
                    shotTable.getFlywheelRpm(tyDegrees), PRE_REV_FLYWHEEL_RPM);
        } else {
            telemetry.addData("Hive visible", "NO (the Limelight needs ~30 s after power-up)");
        }
    }

    private void addShotTelemetry() {
        telemetry.addData("Hive visible / tx", "%b / %.1f",
                visionSubsystem.hasTarget(), visionSubsystem.getTxDegrees());
        telemetry.addData("Turret angle / on target", "%.0f / %b",
                turretSubsystem.getAngleDegrees(), turretSubsystem.isOnTarget());
        telemetry.addData("Flywheel RPM (target / actual)", "%.0f / %.0f",
                shooterSubsystem.getTargetFlywheelRpm(), shooterSubsystem.getFlywheelRpm());
        telemetry.addData("Gate / Intake", "%s / %s",
                shooterServoSubsystem.getState(), intakeSubsystem.getState());
    }

    private void addRunLogTelemetry() {
        telemetry.addLine("--- RUN LOG ---");
        if (lastShotNumber >= 1) {
            telemetry.addData("Shot 1", shotResults[0]);
        }
        if (routine == Routine.THREE_VOLLEY) {
            telemetry.addData("Flower A", flowerAResult);
            telemetry.addData("Shot 2", shotResults[1]);
            telemetry.addData("Flower B", flowerBResult);
            telemetry.addData("Shot 3", shotResults[2]);
        }
        telemetry.addData("Drives that timed out", legTimeouts);
    }

    private void addPoseTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pose X / Y / heading", "%.1f / %.1f / %.0f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
