package org.firstinspires.ftc.teamcode.testingOps;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BiobuzzTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlowerPickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * 2 Volley Wall Flower (Mirror), Alpha
 * Shoot the preload, drive straight to the RIGHT-WALL flower, pick it, drive back to the start spot,
 * shoot again, park. Stays on the audience half until the park. Same field as ThreeVolleyMirrorAlpha.
 *
 * SHARED CODE: this OpMode is a copy of ThreeVolleyMirrorAlpha (same field spots, same shot, same
 * flower steps, same INIT buttons) with a shorter plan. Some flower and path code below is not used
 * by this plan. A fix to the shot or flower steps there should be copied here too.
 *
 * INIT BUTTONS (gamepad 1). The screen shows what is picked.
 *   A = STEP mode: the robot STOPS after every drive and waits for you to press A.
 *       Use it to measure each stop against tape on the floor.
 *   B = NONSTOP (the default): runs straight through like a real Auto.
 *   X = ACTIONS OFF: DRIVE ONLY. Every SHOOT and INTAKE step is just a pause the same
 *       length as in the visualizer (3 s and 1 s). Nothing shoots, nothing intakes.
 *   Y = ACTIONS ON (the default): real shooting (camera, turret, flywheel, gate, intake) and real
 *       flower picking.
 *   With no buttons pressed it runs as a real Auto: NONSTOP, ACTIONS ON.
 *
 * THE PLAN (ACTIONS OFF = drive only, pauses where the shots and flower are)
 *   1. SHOT 1                 : shoot from the start spot (OFF: pause 3 s).
 *   2. DRIVE_TO_WALL_FLOWER   : straight line to the right-wall flower stop, turning from the rear
 *                               wall (90) to face the right wall (0).
 *   3. FLOWER 2               : bar down (only after it stops), creep FLOWER2_CREEP_INCHES in, pick
 *                               (OFF: pause 1 s). "Flower 2" is the right-wall flower, as in the 3 volley.
 *   4. DRIVE_TO_AUDIENCE_SHOT : straight back to the start spot, turning back to face the rear wall.
 *   5. SHOT 2                 : shoot from the start spot again (OFF: pause 3 s).
 *   6. DRIVE_TO_PARK          : curve up the right wall to park.
 *
 * ONE SHOT, WITH ACTIONS ON (the same sequence as ShootAndParkAuto)
 *   PRE-AIM        : the turret turns to where the hive SHOULD be (from the robot's pose and
 *                    BLUE_HIVE_X/Y_INCHES), because the camera only aims once it sees the hive.
 *   FIND_HIVE      : the turret turns until the camera is locked. The shot table gives the RPM.
 *   WAIT_FOR_SPEED : the flywheel gets to that RPM and holds there, and the turret is on target.
 *   FEED_BALLS     : gate OPENED, the intake feeds, but only while the flywheel is at speed.
 *   LIFT_LAST_BALL : gate LIFTED to push the last ball in, then gate CLOSED.
 *   If the camera never locks or the flywheel never gets to speed, the shot is SKIPPED and the Auto
 *   carries on. Between shots the flywheel keeps spinning at PRE_REV_FLYWHEEL_RPM.
 *
 * ONE FLOWER, WITH ACTIONS ON
 *   1. On the way in, FLOWERn_BAR_DOWN_INCHES before the stop, the flower bar goes DOWN (only if
 *      USE_FLOWER_BAR is true) and the intake starts. (Flower 2 uses 0: its bar hit the flower on the
 *      way in, so it only drops once the robot has stopped.) Gate CLOSED: the balls wait at the gate.
 *   2. FLOWER_BAR_DOWN : the robot waits at the stop until the bar is all the way down.
 *   3. FLOWER_CREEP    : it creeps FLOWERn_CREEP_INCHES straight ahead into the flower, intake on.
 *   4. FLOWER_PICK     : it sits there with the intake on for FLOWER_PICK_SECONDS.
 *   5. The intake stops and the robot drives on to its next shot with the bar STILL DOWN. The bar
 *      goes back UP when that shot starts feeding (the intake starts), or when the shot ends or is
 *      skipped. (FLOWER_BAR_UP is only used when a flower is cut short.)
 *
 * THE FLOWER BAR (USE_FLOWER_BAR). It is ON now. If the robot stops following its curves again,
 * the bar's UP number is wrong (see below) and it is dragging: set USE_FLOWER_BAR = false.
 * WHY IT WAS OFF AT FIRST:
 *   A drive test with the bar held "UP" drove straight instead of following the curve, and with the
 *   bar off it drove fine. So FlowerPickerSubsystem's UP number is wrong. Run FlowerPickerAlpha, put
 *   the real UP and DOWN numbers in FlowerPickerSubsystem, check the bar really goes up at INIT,
 *   then set USE_FLOWER_BAR = true. With it off, the bar hangs loose and may swing out when the robot
 *   brakes. That is expected.
 *
 * THE 30 SECOND LIMIT (ACTIONS ON only)
 *   Once the robot has run SKIP_ACTIONS_AFTER_SECONDS (time waiting for A in step mode does not
 *   count), every shot or flower that has not started is skipped, and one that is running is cut
 *   short, so the robot always has time to park. The run log says what was skipped.
 *
 * MIRRORED, NOT ROTATED, on purpose, to fit our practice field. The Pedro guide's rule 7 says a real
 * Auto should switch sides with a 180 degree rotation. On a full field the flowers are not mirror
 * images, so check that each stop lines up with a flower on the field you run it on.
 *
 * Pedro's Follower owns the drive motors and the Pinpoint, so this OpMode does NOT use
 * DriveSubsystem or PinpointSubsystem.
 *
 * HOW TO RUN IT
 *   1. Put tape on the floor at each stop (the numbers are in "Field spots" below).
 *   2. Robot on the start spot (82.5, 8.9): against the audience wall, facing the rear wall. Leave
 *      it STILL (INIT recalibrates the Pinpoint).
 *   3. Select "3 Volley Mirror - Alpha", press INIT, pick A/B and X/Y. Check the screen.
 *   4. Press START. Keep a finger on STOP.
 *   Start with STEP + ACTIONS OFF. When the path is clean, NONSTOP + ACTIONS OFF and write down the
 *   run time. Then ACTIONS ON. Log every run in TeamCode/docs/biobuzz-test-and-match-log.md.
 *
 * WHAT TO WATCH FOR
 *   - "Last stop error" is how far Pedro THINKS the robot ended from the target. Also measure with a
 *     tape measure: within about 1 inch is a pass. If the screen says 0.3 but the tape says 4, the
 *     problem is the Pinpoint, not the path.
 *   - The robot must not touch the HIVE, the flowers, or the walls on the way. Clear the right side.
 *
 * COORDINATES (Pedro's, same as the visualizer): origin at the bottom-left corner, X to the right,
 * Y up toward the rear wall. Heading 90 faces the rear wall, 0 faces the right wall, counterclockwise
 * (left) is positive.
 *
 * HIDDEN FROM THE DRIVER STATION
 *   Nobody is using this OpMode right now, so it is @Disabled to keep the list short.
 *   The code is all still here. To bring it back, put // in front of the @Disabled line
 *   below and redeploy.
 */
@Disabled
@Autonomous(name = "2 Volley Wall Flower Mirror - Alpha", group = "Testing",
        preselectTeleOp = BiobuzzTeleOp.TELEOP_NAME)
public class TwoVolleyWallFlowerMirrorAlpha extends OpMode {

    // ------------------------------------------------------------------
    // ACTIONS OFF: pause lengths (seconds), the same as the visualizer's wait steps.
    // ------------------------------------------------------------------
    private static final double SHOOT_PAUSE_SECONDS = 3.0;

    // The number of the last shot in this plan. The flywheel stops after it.
    private static final int LAST_SHOT_NUMBER = 2;
    private static final double INTAKE_PAUSE_SECONDS = 1.0;

    // ------------------------------------------------------------------
    // ACTIONS ON: shooting settings (the same numbers as ShootAndParkAuto)
    // ------------------------------------------------------------------
    // How long the gate stays OPENED while the intake feeds balls, in seconds.
    private static final double FEED_SECONDS = 3.0;
    // How long the gate stays LIFTED (with the intake still running) to push the last ball in.
    private static final double GATE_LIFT_SECONDS = 0.5;
    // true  = the intake runs only while the flywheel is at speed (balls go one at a time).
    // false = the intake runs the whole feed no matter what (to check the balls move at all).
    private static final boolean INTAKE_WAITS_FOR_SPEED = true;
    // The flywheel speed to spin at while looking for the hive, and between shots. The shot table's
    // RPM takes over once the camera is locked. It must be 1458 or lower (ShooterSubsystem cuts it).
    // TIP: set it to the INIT screen's "Table RPM from here" (with ACTIONS ON).
    private static final double PRE_REV_FLYWHEEL_RPM = 1300.0;
    // How long to wait for the camera to lock onto the hive before skipping a shot.
    private static final double LOCK_TIMEOUT_SECONDS = 3.0;
    // How long to wait for the flywheel to reach speed (after the camera locks) before skipping.
    private static final double SPIN_UP_TIMEOUT_SECONDS = 6.0;
    // The flywheel counts as "at speed" within this many RPM of its target, held for this long.
    private static final double FLYWHEEL_AT_SPEED_TOLERANCE_RPM = 25.0;
    private static final double FLYWHEEL_AT_SPEED_HOLD_SECONDS = 0.15;
    // Only pick a new turret target when the camera says we are more than this far off, and call
    // the hive LOCKED when tx is this small (degrees).
    private static final double ALIGN_REAIM_THRESHOLD_DEGREES = 1.0;
    private static final double LOCKED_TOLERANCE_DEGREES = 2.0;

    // ------------------------------------------------------------------
    // ACTIONS ON: flower and time settings
    // ------------------------------------------------------------------
    // true  = lower the flower bar at each flower, and hold it UP the rest of the time.
    // false = never touch the flower bar (see "THE FLOWER BAR IS OFF" above).
    private static final boolean USE_FLOWER_BAR = true;
    // How long the robot sits at a flower with the intake on (and the bar down), in seconds.
    // TODO: tune on the robot until it reliably collects the pollen.
    private static final double FLOWER_PICK_SECONDS = 2.5;
    // How far before each flower stop (inches) the bar goes DOWN and the intake starts, while the
    // robot is still driving in, so the bar is already down when it arrives. The bar takes about
    // half a second to move. Bar still up when it arrives? BIGGER. Bar hits the flower? SMALLER.
    private static final double FLOWER1_BAR_DOWN_INCHES = 20.0;
    // Flower 2: 0 = the bar only drops once the robot has stopped (it hit the flower on the way in
    // when it dropped early). Make it bigger only if the bar clears the flower.
    private static final double FLOWER2_BAR_DOWN_INCHES = 0.0;
    // How far the robot creeps straight ahead into each flower (inches) once the bar is down.
    // Not reaching the pollen? Make it bigger. Pushing the flower or hitting the wall? Smaller.
    private static final double FLOWER1_CREEP_INCHES = 5.0;
    private static final double FLOWER2_CREEP_INCHES = 8.0;
    // The most time a creep may take before the pick starts anyway.
    private static final double CREEP_TIMEOUT_SECONDS = 1.5;
    // Once the robot has run this long, shots and flowers are skipped or cut short so it can park.
    // TODO: run NONSTOP, see how long the last drive (to park) takes, set this to about 29 minus that.
    private static final double SKIP_ACTIONS_AFTER_SECONDS = 24.0;

    // The most time one drive step is allowed to take, so a drive that never settles cannot use up
    // the whole Auto. Path 1 is about 125 inches long.
    private static final double LEG_TIMEOUT_SECONDS = 8.0;

    // What the drive does at each stop, after it arrives:
    // true  = Pedro HOLDS the spot (pushes back if the robot is bumped). This is Pedro's normal way.
    // false = the drive motors just STOP until the next drive.
    // First test (2026-11-17): holding at the rear flower stop made the robot SHAKE violently while
    // nothing was touching it, so this is false for now. Find out why with Pedro's Tests -> Hold Test
    // on the AutoTune page: if that shakes too, re-tune Foresight on a fresh battery, then try true.
    private static final boolean HOLD_AT_STOPS = false;

    // ------------------------------------------------------------------
    // Field spots (inches and degrees), from 3-volley-mirror.pp (red-3-volley.pp, x -> 141.5 - x)
    // ------------------------------------------------------------------
    // Start: against the audience wall, facing the rear wall. Shots 1 and 3 are taken from here.
    private static final double START_X_INCHES = 82.5;
    private static final double START_Y_INCHES = 8.9;
    private static final double START_HEADING_DEGREES = 90.0;

    // Stop in front of the rear-wall flower, facing the rear wall.
    private static final double REAR_FLOWER_X_INCHES = 94.3;
    private static final double REAR_FLOWER_Y_INCHES = 123.5;
    private static final double REAR_FLOWER_HEADING_DEGREES = 90.0;

    // The robot creeps FLOWER1_CREEP_INCHES forward from this stop (to y = 126.5) before picking.

    // ACTIONS ON only: the shot 2 spot is this many inches straight back from where the creep at
    // flower 1 ends, still facing the rear wall. More = closer to the hive for the shot.
    private static final double REAR_SHOT_BACK_UP_INCHES = 7.0;

    // Stop in front of the right-wall flower, facing the right wall. 3 inches back from the
    // original 126.6 so the bar can come down clear of the flower. The robot then creeps
    // FLOWER2_CREEP_INCHES forward (back to 126.6) before picking.
    private static final double WALL_FLOWER_X_INCHES = 123.6;
    private static final double WALL_FLOWER_Y_INCHES = 47.2;
    private static final double WALL_FLOWER_HEADING_DEGREES = 0.0;

    // Park up the right wall, facing the rear wall.
    private static final double PARK_X_INCHES = 127.75;
    private static final double PARK_Y_INCHES = 106.1;
    private static final double PARK_HEADING_DEGREES = 90.0;

    // Where the BLUE hive is, for the PRE-AIM only (the camera does the real aiming). Its pivot runs
    // left-right at Y = 70.75, and the blue hive's center is 12.75 inches right of the field's
    // center (the red and blue hives are 25.5 inches apart). Derived from the manual
    // (biobuzz-hive-apriltag-geometry.md), not measured.
    private static final double BLUE_HIVE_X_INCHES = 83.5;
    private static final double BLUE_HIVE_Y_INCHES = 70.75;

    // Curve control points (inches), from 3-volley-mirror.pp. A control point bends the path toward
    // it. The robot does NOT drive through it.
    // Path 1 (start -> rear flower): two control points, it swings right, away from the HIVE.
    private static final double PATH1_CONTROL1_X = 126.0;
    private static final double PATH1_CONTROL1_Y = 15.0;
    private static final double PATH1_CONTROL2_X = 126.0;
    private static final double PATH1_CONTROL2_Y = 110.0;
    // Path 2 (-> right-wall flower): one control point.
    private static final double PATH2_CONTROL_X = 122.0;
    private static final double PATH2_CONTROL_Y = 105.0;
    // Path 3 (right-wall flower -> audience shot) is a straight line, so it has none.
    // Path 4 (audience shot -> park): one control point.
    private static final double PATH4_CONTROL_X = 119.5;
    private static final double PATH4_CONTROL_Y = 30.0;

    // ------------------------------------------------------------------
    // The steps. The robot is always in exactly one of these.
    // ------------------------------------------------------------------
    private enum AutoState {
        // One shot (used 3 times, shotNumber says which)
        SHOT_PAUSE,             // ACTIONS OFF: just a pause
        SHOT_FIND_HIVE,         // ACTIONS ON from here down
        SHOT_WAIT_FOR_SPEED,
        SHOT_FEED_BALLS,
        SHOT_LIFT_LAST_BALL,
        // One flower (used 2 times, flowerNumber says which)
        FLOWER_PAUSE,           // ACTIONS OFF: just a pause
        FLOWER_BAR_DOWN,        // ACTIONS ON from here down
        FLOWER_CREEP,
        FLOWER_PICK,
        FLOWER_BAR_UP,
        // Drives
        DRIVE_TO_REAR_FLOWER,
        DRIVE_TO_REAR_SHOT,     // ACTIONS ON only
        DRIVE_TO_WALL_FLOWER,
        DRIVE_TO_AUDIENCE_SHOT,
        DRIVE_TO_PARK,
        WAIT_FOR_BUTTON,        // STEP mode only: stopped after a drive, waiting for A
        DONE
    }

    // ------------------------------------------------------------------
    // Things this OpMode uses
    // ------------------------------------------------------------------
    private Follower follower;
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;
    private FlowerPickerSubsystem flowerPickerSubsystem;   // null when USE_FLOWER_BAR is false
    private ShotTable shotTable;

    // A PoseFactory makes poses (an X, a Y and a heading) from numbers in DEGREES.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Pose startPose;
    private Pose rearFlowerPose;
    private Pose rearFlowerCreepPose;   // where the creep at flower 1 ends
    private Pose rearShotPose;        // the shot 2 spot, facing the rear wall
    private Pose wallFlowerPose;
    private Pose wallFlowerCreepPose;   // where the creep at flower 2 ends
    private Pose parkPose;

    // The paths are built once in init(), never inside loop().
    private Path toRearFlowerPath;
    private Path toRearShotPath;                 // ACTIONS ON only
    private Path wallFlowerFromRearFlowerPath;   // ACTIONS OFF: the path exactly as drawn
    private Path wallFlowerFromRearShotPath;     // ACTIONS ON: starts at the shot 2 spot
    private Path rearFlowerCreepPath;            // ACTIONS ON: the creep into flower 1
    private Path wallFlowerCreepPath;            // ACTIONS ON: the creep into flower 2
    private Path toAudienceShotPath;             // ACTIONS OFF: starts at the flower 2 stop
    private Path audienceShotFromCreepPath;      // ACTIONS ON: starts where the flower 2 creep ended
    private Path toParkPath;
    private Path startToWallFlowerPath;          // start spot -> right-wall flower stop

    // Picked in INIT.
    // Defaults: NONSTOP with ACTIONS ON, ready for a match. (In a match nobody can press A, so a
    // STEP-mode Auto would stop after its first drive.)
    private boolean stepMode = false;     // A = true, B = false
    private boolean actionsOn = true;     // Y = true, X = false

    private AutoState autoState = AutoState.DONE;
    private int shotNumber = 0;     // 1, 2 or 3
    private int flowerNumber = 0;   // 1 or 2
    // True from when the bar goes down on the way in to a flower until it goes back up.
    private boolean barIsDownForFlower = false;

    // STEP mode: which drive just finished (so we know what comes next when A is pressed).
    private AutoState driveThatFinished = AutoState.DONE;
    // Remembers if A was down last loop, so one press counts once.
    private boolean aWasPressed = false;

    // Times the current step. A timer is used instead of sleep() so loop() keeps running (and the
    // Follower keeps holding the robot's position).
    private final ElapsedTime stateTimer = new ElapsedTime();
    // Times one drive step (see LEG_TIMEOUT_SECONDS).
    private final ElapsedTime legTimer = new ElapsedTime();
    // Times the whole run, from START.
    private final ElapsedTime runTimer = new ElapsedTime();
    // Time spent waiting for A in step mode. Subtracted so the run time is robot time only.
    private final ElapsedTime buttonWaitTimer = new ElapsedTime();
    private double totalButtonWaitSeconds = 0.0;
    private double finishedRunSeconds = 0.0;

    // The flywheel RPM the shot table gave us when the camera locked, kept for the whole shot.
    private double shotFlywheelRpm = 0.0;
    // The flywheel-only "at speed" check (same as ShootAndParkAuto). Worked out once per loop.
    private boolean flywheelReady = false;
    private boolean flywheelWasInTolerance = false;
    private final ElapsedTime flywheelHoldTimer = new ElapsedTime();
    // For counting flywheel dips during a feed (about one per ball shot).
    private boolean feedWasReady = false;
    private int flywheelDipCount = 0;
    private double lockedAtSeconds = 0.0;

    // The run log. It stays on the screen after the run.
    private final String[] shotResults = {"not yet", "not yet", "not yet"};
    private final String[] flowerResults = {"not yet", "not yet"};
    private String legTimeouts = "none";
    private String lastStopName = "none yet";
    private double lastStopErrorInches = 0.0;
    private double lastStopHeadingErrorDegrees = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Build the poses.
        startPose = poseFactory.of(START_X_INCHES, START_Y_INCHES, START_HEADING_DEGREES);
        rearFlowerPose = poseFactory.of(REAR_FLOWER_X_INCHES, REAR_FLOWER_Y_INCHES, REAR_FLOWER_HEADING_DEGREES);
        wallFlowerPose = poseFactory.of(WALL_FLOWER_X_INCHES, WALL_FLOWER_Y_INCHES, WALL_FLOWER_HEADING_DEGREES);
        rearFlowerCreepPose = poseAhead(rearFlowerPose, FLOWER1_CREEP_INCHES);
        // Shot 2 spot: straight back from the creep end, still facing the rear wall.
        rearShotPose = poseAhead(rearFlowerCreepPose, -REAR_SHOT_BACK_UP_INCHES);
        wallFlowerCreepPose = poseAhead(wallFlowerPose, FLOWER2_CREEP_INCHES);
        parkPose = poseFactory.of(PARK_X_INCHES, PARK_Y_INCHES, PARK_HEADING_DEGREES);

        // Create the Follower. This recalibrates the Pinpoint, so keep the robot still.
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        // The subsystems are always created, so ACTIONS can be turned on during INIT. With ACTIONS
        // OFF they are never told to do anything.
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        // The gate moves to CLOSED here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        // The turret does not move at INIT. It only starts reading its encoder.
        turretSubsystem = new TurretSubsystem(hardwareMap);
        // The Limelight starts polling here. It needs about 30 seconds to boot after power-up.
        visionSubsystem = new VisionSubsystem(hardwareMap);
        visionSubsystem.setAlliance(VisionSubsystem.Alliance.BLUE);
        shotTable = new ShotTable();
        if (USE_FLOWER_BAR) {
            // The bar moves UP here. (Rule R103.B allows an Auto to set servos during INIT.)
            flowerPickerSubsystem = new FlowerPickerSubsystem(hardwareMap);
        }

        toRearFlowerPath = buildToRearFlowerPath();
        toRearShotPath = buildToRearShotPath();
        wallFlowerFromRearFlowerPath = buildWallFlowerFromRearFlowerPath();
        wallFlowerFromRearShotPath = buildWallFlowerFromRearShotPath();
        toAudienceShotPath = buildToAudienceShotPath();
        rearFlowerCreepPath = buildCreepPath(rearFlowerPose, rearFlowerCreepPose);
        wallFlowerCreepPath = buildCreepPath(wallFlowerPose, wallFlowerCreepPose);
        audienceShotFromCreepPath = buildAudienceShotFromCreepPath();
        toParkPath = buildToParkPath();
        startToWallFlowerPath = Paths.line(startPose, wallFlowerPose).linear(startPose, wallFlowerPose);

        telemetry.addLine("Pinpoint ready. Keep the robot still.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // A = step, B = nonstop. X = actions off, Y = actions on.
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

        // The Follower is idle here, so this only reads the Pinpoint and keeps the wheels stopped.
        follower.update();
        visionSubsystem.update();
        // The turret is NOT driven during INIT (nothing may move). We only read it.
        turretSubsystem.update();

        addSafetyTelemetry();
        telemetry.addData("Mode (A = step, B = nonstop)", stepMode ? "STEP" : "NONSTOP");
        telemetry.addData("Actions (Y = on, X = off)",
                actionsOn ? "ON: shoot + intake" : "OFF: drive only, pauses");
        if (actionsOn) {
            telemetry.addData("Hive", "BLUE");
            telemetry.addData("Flower bar", USE_FLOWER_BAR ? "ON" : "OFF (intake only at flowers)");
            addInitCameraTelemetry();
        }
        addPoseTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        // The robot has not moved since INIT. Tell Pedro again, in case it was nudged during setup.
        follower.setPose(startPose);
        follower.update();

        // A is probably not held right now, but if it is, ignore it until it is let go.
        aWasPressed = gamepad1.a;

        runTimer.reset();
        totalButtonWaitSeconds = 0.0;
        flywheelReady = false;
        flywheelWasInTolerance = false;

        if (actionsOn) {
            // Start warming up the flywheel right away.
            shooterSubsystem.setTargetRpm(PRE_REV_FLYWHEEL_RPM);
        }
        startShot(1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors. Pedro reads the Pinpoint and sets the wheel powers. It must run every
        //    loop, or the robot does not move (or stops steering).
        follower.update();
        visionSubsystem.update();

        // A "new press" is true only on the one loop where A goes from up to down.
        boolean aNewPress = gamepad1.a && !aWasPressed;
        aWasPressed = gamepad1.a;

        // Keep the flower bar UP, except while a flower is being picked.
        if (USE_FLOWER_BAR && autoState != AutoState.FLOWER_PICK && !barIsDownForFlower) {
            flowerPickerSubsystem.goToUp();
        }

        // ACTIONS ON and out of time? Cut a running shot or flower short so the robot can still park.
        if (actionsOn && isOutOfTime()) {
            cutShortIfBusy();
        }

        // 2. Decide what to do next, and tell the subsystems.
        switch (autoState) {
            // ---------------- ONE SHOT ----------------
            case SHOT_PAUSE:
                if (stateTimer.seconds() >= SHOOT_PAUSE_SECONDS) {
                    shotResults[shotNumber - 1] = "pause (actions off)";
                    finishShot();
                }
                break;

            case SHOT_FIND_HIVE:
                // Warm the flywheel up while the turret turns until the camera is centered on the hive.
                shooterSubsystem.setTargetRpm(PRE_REV_FLYWHEEL_RPM);
                aimTurretAtHive();

                if (isHiveLocked()) {
                    // Locked. Ask the shot table how fast to shoot from here, and keep that number.
                    lockedAtSeconds = robotSeconds();
                    shotFlywheelRpm = shotTable.getFlywheelRpm(visionSubsystem.getTyDegrees());
                    shooterSubsystem.setTargetRpm(shotFlywheelRpm);
                    stateTimer.reset();
                    autoState = AutoState.SHOT_WAIT_FOR_SPEED;
                } else if (stateTimer.seconds() >= LOCK_TIMEOUT_SECONDS) {
                    // No camera lock: shoot anyway at the pre-rev speed, from the pre-aim angle.
                    // (The run log then says "locked at -1.0 s".)
                    lockedAtSeconds = -1.0;
                    shotFlywheelRpm = PRE_REV_FLYWHEEL_RPM;
                    stateTimer.reset();
                    autoState = AutoState.SHOT_WAIT_FOR_SPEED;
                }
                break;

            case SHOT_WAIT_FOR_SPEED:
                // Same number every loop is safe: ShooterSubsystem ignores a repeat request.
                shooterSubsystem.setTargetRpm(shotFlywheelRpm);
                aimTurretAtHive();

                // The firing gate: the flywheel at speed (and held there) AND the turret on target.
                // (A shot without the camera, lockedAtSeconds < 0, does not wait for the turret.)
                if (flywheelReady && (turretSubsystem.isOnTarget() || lockedAtSeconds < 0)) {
                    // The feed (and the intake) starts now, so the flower bar can go back UP.
                    barIsDownForFlower = false;
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
                // The gate is OPENED. Feed one ball at a time: the intake runs only while the
                // flywheel is at speed.
                if (flywheelReady || !INTAKE_WAITS_FOR_SPEED) {
                    intakeSubsystem.intake();
                } else {
                    intakeSubsystem.stop();
                }

                // Every time the flywheel drops out of "at speed", a ball has just gone through.
                if (feedWasReady && !flywheelReady) {
                    flywheelDipCount = flywheelDipCount + 1;
                }
                feedWasReady = flywheelReady;

                if (stateTimer.seconds() >= FEED_SECONDS) {
                    shooterServoSubsystem.goToLifted();
                    stateTimer.reset();
                    autoState = AutoState.SHOT_LIFT_LAST_BALL;
                }
                break;

            case SHOT_LIFT_LAST_BALL:
                // The gate is LIFTED to push the last ball in. The intake keeps running.
                intakeSubsystem.intake();

                if (stateTimer.seconds() >= GATE_LIFT_SECONDS) {
                    shotResults[shotNumber - 1] = String.format(
                            "SHOT (locked at %.1f s, %.0f RPM, %d dips)",
                            lockedAtSeconds, shotFlywheelRpm, flywheelDipCount);
                    finishShot();
                }
                break;

            // ---------------- ONE FLOWER ----------------
            case FLOWER_PAUSE:
                if (stateTimer.seconds() >= INTAKE_PAUSE_SECONDS) {
                    flowerResults[flowerNumber - 1] = "pause (actions off)";
                    finishFlower();
                }
                break;

            case FLOWER_BAR_DOWN:
                // Bar DOWN (if it is on) and the intake pulling. Wait until the bar is all the way
                // down, then creep into the flower.
                if (USE_FLOWER_BAR) {
                    flowerPickerSubsystem.goToDown();
                }
                intakeSubsystem.intake();
                if (!USE_FLOWER_BAR || flowerPickerSubsystem.isAtPosition()) {
                    legTimer.reset();
                    follower.follow(flowerNumber == 1 ? rearFlowerCreepPath : wallFlowerCreepPath);
                    autoState = AutoState.FLOWER_CREEP;
                }
                break;

            case FLOWER_CREEP:
                // Creeping straight ahead into the flower, bar down and intake on.
                if (USE_FLOWER_BAR) {
                    flowerPickerSubsystem.goToDown();
                }
                intakeSubsystem.intake();
                if (!follower.isBusy() || legTimer.seconds() >= CREEP_TIMEOUT_SECONDS) {
                    if (!HOLD_AT_STOPS) {
                        follower.stop();
                    }
                    stateTimer.reset();
                    autoState = AutoState.FLOWER_PICK;
                }
                break;

            case FLOWER_PICK:
                // Bar DOWN (if it is on) and the intake pulling. The gate is CLOSED, so the balls
                // wait at the gate for the next shot.
                if (USE_FLOWER_BAR) {
                    flowerPickerSubsystem.goToDown();
                }
                intakeSubsystem.intake();

                if (stateTimer.seconds() >= FLOWER_PICK_SECONDS) {
                    flowerResults[flowerNumber - 1] = String.format("picked for %.1f s", FLOWER_PICK_SECONDS);
                    // Keep the bar DOWN (barIsDownForFlower stays true). It goes back UP when the
                    // next shot starts feeding. Stop the intake and drive on.
                    intakeSubsystem.stop();
                    finishFlower();
                }
                break;

            case FLOWER_BAR_UP:
                // The bar is going UP (the goToUp() near the top of loop() sends it). Keep the
                // intake on until it is up, so nothing falls back out, then drive on.
                intakeSubsystem.intake();
                if (!USE_FLOWER_BAR || flowerPickerSubsystem.isAtPosition()) {
                    intakeSubsystem.stop();
                    finishFlower();
                }
                break;

            // ---------------- DRIVES ----------------
            case DRIVE_TO_REAR_FLOWER:
                lowerBarNearFlower(FLOWER1_BAR_DOWN_INCHES);
                if (isDriveDone("rear flower")) {
                    finishDrive("Rear flower", rearFlowerPose, AutoState.DRIVE_TO_REAR_FLOWER);
                }
                break;

            case DRIVE_TO_REAR_SHOT:
                if (isDriveDone("rear shot")) {
                    finishDrive("Rear shot", rearShotPose, AutoState.DRIVE_TO_REAR_SHOT);
                }
                break;

            case DRIVE_TO_WALL_FLOWER:
                lowerBarNearFlower(FLOWER2_BAR_DOWN_INCHES);
                if (isDriveDone("wall flower")) {
                    finishDrive("Wall flower", wallFlowerPose, AutoState.DRIVE_TO_WALL_FLOWER);
                }
                break;

            case DRIVE_TO_AUDIENCE_SHOT:
                if (isDriveDone("audience shot")) {
                    finishDrive("Audience shot", startPose, AutoState.DRIVE_TO_AUDIENCE_SHOT);
                }
                break;

            case DRIVE_TO_PARK:
                if (isDriveDone("park")) {
                    finishedRunSeconds = robotSeconds();
                    finishDrive("Park", parkPose, AutoState.DRIVE_TO_PARK);
                }
                break;

            case WAIT_FOR_BUTTON:
                // STEP mode: the robot waits here until A is pressed (held by Pedro only if
                // HOLD_AT_STOPS is true).
                if (aNewPress) {
                    totalButtonWaitSeconds = totalButtonWaitSeconds + buttonWaitTimer.seconds();
                    continueAfterDrive(driveThatFinished);
                }
                break;

            case DONE:
                // Nothing left to do. (The Follower holds the last pose only if HOLD_AT_STOPS.)
                break;
        }

        // The subsystems need update() once per loop, AFTER they get their commands. Then work out
        // the flywheel-only "at speed" answer from the fresh reading. With ACTIONS OFF they have no
        // commands, so they stay stopped.
        shooterSubsystem.update();
        turretSubsystem.update();
        intakeSubsystem.update();
        updateFlywheelReady();

        // 3. Telemetry
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
        telemetry.addData("Last stop", lastStopName);
        telemetry.addData("Last stop error (in)", "%.2f", lastStopErrorInches);
        telemetry.addData("Last stop heading error (deg)", "%.1f", lastStopHeadingErrorDegrees);
        if (actionsOn) {
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
        // Put the Follower in IDLE and tell the drivetrain to stop right now. (follower.stop()
        // only changes the mode. The wheels are zeroed by the next update(), and there is none.)
        follower.stop();
        follower.drivetrain.stop();

        shooterSubsystem.stop();
        intakeSubsystem.stop();
        // The turret floats. The hub keeps its encoder count, so the TeleOp knows its angle.
        turretSubsystem.stop();
        visionSubsystem.stop();
    }

    // ------------------------------------------------------------------
    // Shots
    // ------------------------------------------------------------------

    // Start shot 1, 2 or 3: a pause (ACTIONS OFF), or the real shot (ACTIONS ON).
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
        // PRE-AIM: point the turret where the hive should be, so the camera can find it.
        turretSubsystem.setTargetAngle(turretAngleToHiveDegrees());
        autoState = AutoState.SHOT_FIND_HIVE;
    }

    // A shot could not happen. Write down why, then carry on as if it were done.
    private void skipShot(String reason) {
        shotResults[shotNumber - 1] = reason;
        finishShot();
    }

    // The shot is over (done, skipped or a pause). Go to the next step.
    private void finishShot() {
        // The bar is never down after a shot (it may still be down if the shot was skipped).
        barIsDownForFlower = false;
        if (actionsOn) {
            shooterServoSubsystem.goToClosed();
            intakeSubsystem.stop();
            // Hold the turret where it is while we drive, so it does not swing around.
            turretSubsystem.holdCurrentAngle();
            if (shotNumber < LAST_SHOT_NUMBER) {
                shooterSubsystem.setTargetRpm(PRE_REV_FLYWHEEL_RPM);   // keep it spinning
            } else {
                shooterSubsystem.stop();                               // last shot: stop the wheels
            }
        }

        // This plan: shot 1 -> right-wall flower, shot 2 -> park.
        if (shotNumber == 1) {
            startDrive(startToWallFlowerPath, AutoState.DRIVE_TO_WALL_FLOWER);
        } else {
            startDrive(toParkPath, AutoState.DRIVE_TO_PARK);
        }
    }

    // The turret angle that points at the hive from where the robot is now. Turret angles are 0
    // straight ahead and positive to the LEFT, so this is "the direction to the hive" minus "the
    // direction the robot faces", wrapped to -180..180. TurretSubsystem clips it to +/-120.
    private double turretAngleToHiveDegrees() {
        Pose pose = follower.pose();
        double toHiveDegrees = Math.toDegrees(
                Math.atan2(BLUE_HIVE_Y_INCHES - pose.y(), BLUE_HIVE_X_INCHES - pose.x()));
        return wrapDegrees(toHiveDegrees - Math.toDegrees(pose.heading()));
    }

    // ------------------------------------------------------------------
    // Flowers
    // ------------------------------------------------------------------

    // Start flower 1 or 2: a pause (ACTIONS OFF), or the real pick (ACTIONS ON).
    private void startFlower(int number) {
        flowerNumber = number;
        stateTimer.reset();
        if (!actionsOn) {
            autoState = AutoState.FLOWER_PAUSE;
            return;
        }
        if (isOutOfTime()) {
            // The bar may have gone down on the way in. Put it back up before driving on.
            barIsDownForFlower = false;
            flowerResults[flowerNumber - 1] = "SKIPPED: out of time";
            finishFlower();
            return;
        }
        // The gate must be CLOSED so the balls we pick up wait at the gate. The bar goes (or stays)
        // DOWN from here until FLOWER_BAR_UP.
        shooterServoSubsystem.goToClosed();
        barIsDownForFlower = true;
        autoState = AutoState.FLOWER_BAR_DOWN;
    }

    // ACTIONS ON: call every loop while driving TO a flower. Once the robot is within
    // barDownInches of the stop (FLOWER1_ or FLOWER2_BAR_DOWN_INCHES), the bar goes DOWN and the intake starts, so they are
    // ready when the robot arrives. The gate stays CLOSED, so the balls wait at the gate.
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

    // Done picking: the bar goes back UP (loop() keeps sending goToUp() once we leave FLOWER_PICK).
    private void startFlowerBarUp() {
        barIsDownForFlower = false;
        if (USE_FLOWER_BAR) {
            flowerPickerSubsystem.goToUp();
        }
        autoState = AutoState.FLOWER_BAR_UP;
    }

    // The flower is over (done, skipped or a pause). Go to the next step.
    private void finishFlower() {
        if (actionsOn) {
            intakeSubsystem.stop();
        }
        if (flowerNumber == 1) {
            if (actionsOn) {
                // Back up and turn so the turret can see the hive for shot 2.
                startDrive(toRearShotPath, AutoState.DRIVE_TO_REAR_SHOT);
            } else {
                // The path as drawn: shot 2 (a pause) right here at the rear flower.
                startShot(2);
            }
        } else {
            // ACTIONS ON starts where the creep ended. ACTIONS OFF starts at the flower 2 stop.
            startDrive(actionsOn ? audienceShotFromCreepPath : toAudienceShotPath,
                    AutoState.DRIVE_TO_AUDIENCE_SHOT);
        }
    }

    // ------------------------------------------------------------------
    // Running out of time (ACTIONS ON only)
    // ------------------------------------------------------------------
    // Robot time since START, not counting time spent waiting for A in step mode.
    private double robotSeconds() {
        double waitingNow = autoState == AutoState.WAIT_FOR_BUTTON ? buttonWaitTimer.seconds() : 0.0;
        return runTimer.seconds() - totalButtonWaitSeconds - waitingNow;
    }

    private boolean isOutOfTime() {
        return robotSeconds() >= SKIP_ACTIONS_AFTER_SECONDS;
    }

    // If a shot or flower is running, end it now (gate closed, intake off, bar up) and move on.
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
                flowerResults[flowerNumber - 1] = "CUT SHORT: out of time";
                startFlowerBarUp();
                break;
            default:
                // Driving, a pause, waiting for A, or done: nothing to cut.
                break;
        }
    }

    // ------------------------------------------------------------------
    // Drives
    // ------------------------------------------------------------------
    private void startDrive(Path path, AutoState driveState) {
        if (!Constants.FORESIGHT_TUNED) {
            // Do not drive on another robot's braking numbers. Stop the wheels, we are done.
            shooterSubsystem.stop();
            autoState = AutoState.DONE;
            return;
        }
        legTimer.reset();
        follower.follow(path);
        autoState = driveState;
    }

    // True once the drive has arrived and settled, or has taken too long.
    private boolean isDriveDone(String driveName) {
        // isBusy() stays true until the robot has arrived and settled.
        if (!follower.isBusy()) {
            return true;
        }
        if (legTimer.seconds() >= LEG_TIMEOUT_SECONDS) {
            legTimeouts = legTimeouts.equals("none") ? driveName : legTimeouts + ", " + driveName;
            return true;
        }
        return false;
    }

    // Called once when a drive has arrived. Records how close it got, then either waits for A
    // (STEP mode) or goes straight on.
    private void finishDrive(String stopName, Pose targetPose, AutoState driveState) {
        if (!HOLD_AT_STOPS) {
            // Stop holding: the Follower goes IDLE, and its next update() stops the drive motors.
            // The next follow() starts it up again.
            follower.stop();
        }

        Pose pose = follower.pose();
        lastStopName = stopName;
        lastStopErrorInches = Math.hypot(pose.x() - targetPose.x(), pose.y() - targetPose.y());
        lastStopHeadingErrorDegrees = wrapDegrees(Math.toDegrees(pose.heading() - targetPose.heading()));

        if (stepMode && driveState != AutoState.DRIVE_TO_PARK) {
            driveThatFinished = driveState;
            buttonWaitTimer.reset();
            autoState = AutoState.WAIT_FOR_BUTTON;
        } else {
            continueAfterDrive(driveState);
        }
    }

    // What comes after each drive.
    private void continueAfterDrive(AutoState driveState) {
        switch (driveState) {
            case DRIVE_TO_REAR_FLOWER:
                startFlower(1);
                break;
            case DRIVE_TO_REAR_SHOT:
                startShot(2);
                break;
            case DRIVE_TO_WALL_FLOWER:
                startFlower(2);
                break;
            case DRIVE_TO_AUDIENCE_SHOT:
                startShot(2);
                break;
            default:
                autoState = AutoState.DONE;
                break;
        }
    }

    // ------------------------------------------------------------------
    // Aiming and the flywheel (same as ShootAndParkAuto)
    // ------------------------------------------------------------------
    /*
     * Turns the turret toward the hive. Call it every loop while we are looking and waiting.
     */
    private void aimTurretAtHive() {
        if (!visionSubsystem.hasTarget()) {
            // The camera does not see the hive. Leave the turret where it is.
            return;
        }

        double txDegrees = visionSubsystem.getTxDegrees();

        // Pick a new target if the turret is not in POSITION mode yet, or if the camera says we are
        // far enough off. Picking a new target every loop would keep the turret from settling.
        boolean notAimingYet = turretSubsystem.getState() != TurretSubsystem.TurretState.POSITION;
        boolean farOff = Math.abs(txDegrees) > ALIGN_REAIM_THRESHOLD_DEGREES;
        if (notAimingYet || farOff) {
            // tx is positive to the RIGHT, but turret angles are positive to the LEFT, so subtract.
            double newTargetAngle = turretSubsystem.getAngleDegrees() - txDegrees;
            turretSubsystem.setTargetAngle(newTargetAngle);
        }
    }

    /*
     * True when the camera sees the hive, the hive is nearly centered, and the turret is on target.
     */
    private boolean isHiveLocked() {
        return visionSubsystem.hasTarget()
                && Math.abs(visionSubsystem.getTxDegrees()) <= LOCKED_TOLERANCE_DEGREES
                && turretSubsystem.isOnTarget();
    }

    /*
     * Works out flywheelReady: true when the FLYWHEEL (only) has been within
     * FLYWHEEL_AT_SPEED_TOLERANCE_RPM of its target for FLYWHEEL_AT_SPEED_HOLD_SECONDS. Call it once
     * per loop, right after shooterSubsystem.update().
     */
    private void updateFlywheelReady() {
        double targetRpm = shooterSubsystem.getTargetFlywheelRpm();
        double errorRpm = Math.abs(shooterSubsystem.getFlywheelRpm() - targetRpm);

        // A target of 0 means the shooter is stopped, so it can never be "at speed".
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

    // Wraps an angle to -180..180 degrees, so 359 and 1 count as 2 degrees apart, not 358.
    private double wrapDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees = degrees - 360.0;
        }
        while (degrees < -180.0) {
            degrees = degrees + 360.0;
        }
        return degrees;
    }

    // ------------------------------------------------------------------
    // Paths
    // ------------------------------------------------------------------
    // Paths.curve(start, control points..., end) makes a curved path. Paths.line(start, end) makes
    // a straight one. Then the heading rule:
    //   .constant(pose)     : keep facing that pose's heading the whole way.
    //   .linear(start, end) : turn smoothly from the start heading to the end heading, the short way.

    // 1. Start -> rear flower. Keeps facing the rear wall (90) the whole way.
    private Path buildToRearFlowerPath() {
        Pose control1 = new Pose(PATH1_CONTROL1_X, PATH1_CONTROL1_Y);
        Pose control2 = new Pose(PATH1_CONTROL2_X, PATH1_CONTROL2_Y);
        return Paths.curve(startPose, control1, control2, rearFlowerPose).constant(rearFlowerPose);
    }

    // ACTIONS ON only: end of the flower 1 creep -> straight back REAR_SHOT_BACK_UP_INCHES, still
    // facing the rear wall (90).
    private Path buildToRearShotPath() {
        return Paths.line(rearFlowerCreepPose, rearShotPose).constant(rearFlowerCreepPose);
    }

    // ACTIONS ON: a short straight creep from a flower stop into the flower, facing the same way.
    private Path buildCreepPath(Pose stopPose, Pose creepPose) {
        return Paths.line(stopPose, creepPose).constant(stopPose);
    }

    // ACTIONS ON: end of the flower 2 creep -> audience shot spot (the start). Straight line,
    // turning from 0 to 90.
    private Path buildAudienceShotFromCreepPath() {
        return Paths.line(wallFlowerCreepPose, startPose).linear(wallFlowerCreepPose, startPose);
    }

    // The pose a given number of inches straight ahead of a pose, facing the same way.
    private Pose poseAhead(Pose pose, double inches) {
        return new Pose(pose.x() + inches * Math.cos(pose.heading()),
                pose.y() + inches * Math.sin(pose.heading()),
                pose.heading());
    }

    // 2. (ACTIONS OFF, as drawn) Rear flower -> right-wall flower. Turns from the rear wall (90) to
    //    the right wall (0) while it drives.
    private Path buildWallFlowerFromRearFlowerPath() {
        Pose control = new Pose(PATH2_CONTROL_X, PATH2_CONTROL_Y);
        return Paths.curve(rearFlowerPose, control, wallFlowerPose).linear(rearFlowerPose, wallFlowerPose);
    }

    // 2. (ACTIONS ON) Rear shot spot -> right-wall flower, same control point. Turns from the rear
    //    wall (90) to the right wall (0).
    private Path buildWallFlowerFromRearShotPath() {
        Pose control = new Pose(PATH2_CONTROL_X, PATH2_CONTROL_Y);
        return Paths.curve(rearShotPose, control, wallFlowerPose).linear(rearShotPose, wallFlowerPose);
    }

    // 3. Right-wall flower -> audience shot spot (the start). Straight line, turning from 0 to 90.
    private Path buildToAudienceShotPath() {
        return Paths.line(wallFlowerPose, startPose).linear(wallFlowerPose, startPose);
    }

    // 4. Audience shot -> park. Keeps facing the rear wall (90) the whole way.
    private Path buildToParkPath() {
        Pose control = new Pose(PATH4_CONTROL_X, PATH4_CONTROL_Y);
        return Paths.curve(startPose, control, parkPose).constant(parkPose);
    }

    // ------------------------------------------------------------------
    // Telemetry helpers
    // ------------------------------------------------------------------
    private void addSafetyTelemetry() {
        if (Constants.FORESIGHT_TUNED) {
            telemetry.addData("Foresight", "tuned, OK to drive");
        } else {
            telemetry.addLine("!!! NOT TUNED: this OpMode will NOT drive. !!!");
            telemetry.addLine("Tune Foresight (see the Pedro guide), then set FORESIGHT_TUNED = true.");
        }
    }

    // Checks to read BEFORE START (ACTIONS ON): does the camera see the hive from the start spot?
    private void addInitCameraTelemetry() {
        telemetry.addData("Camera connected", visionSubsystem.isCameraConnected());
        if (visionSubsystem.hasTarget()) {
            double tyDegrees = visionSubsystem.getTyDegrees();
            telemetry.addData("Hive visible", "YES (ty %.1f)", tyDegrees);
            telemetry.addData("Table RPM from here", "%.0f  (PRE_REV_FLYWHEEL_RPM is %.0f)",
                    shotTable.getFlywheelRpm(tyDegrees), PRE_REV_FLYWHEEL_RPM);
            if (!shotTable.isInRange(tyDegrees)) {
                telemetry.addLine("Warning: ty is OUTSIDE the shot table. It uses the nearest row.");
            }
        } else {
            telemetry.addData("Hive visible", "NO (the Limelight needs ~30 s after power-up)");
        }
    }

    // What the shooter is doing right now (ACTIONS ON).
    private void addShotTelemetry() {
        telemetry.addData("Hive visible", visionSubsystem.hasTarget());
        telemetry.addData("tx (deg)", "%.1f", visionSubsystem.getTxDegrees());
        telemetry.addData("Turret angle / on target", "%.0f / %b",
                turretSubsystem.getAngleDegrees(), turretSubsystem.isOnTarget());
        telemetry.addData("Flywheel RPM (target / actual)", "%.0f / %.0f",
                shooterSubsystem.getTargetFlywheelRpm(), shooterSubsystem.getFlywheelRpm());
        telemetry.addData("FLYWHEEL at speed", flywheelReady);
        telemetry.addData("Gate", shooterServoSubsystem.getState());
        telemetry.addData("Intake", intakeSubsystem.getState());
    }

    // The run log. It fills in as the run goes and stays on the screen after it ends.
    private void addRunLogTelemetry() {
        telemetry.addLine("--- RUN LOG ---");
        telemetry.addData("Shot 1", shotResults[0]);
        telemetry.addData("Wall flower", flowerResults[1]);
        telemetry.addData("Shot 2", shotResults[1]);
        telemetry.addData("Drives that timed out", legTimeouts);
    }

    private void addPoseTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pose X (in)", "%.2f", pose.x());
        telemetry.addData("Pose Y (in)", "%.2f", pose.y());
        telemetry.addData("Pose heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
    }
}
