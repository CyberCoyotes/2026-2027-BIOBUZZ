package org.firstinspires.ftc.teamcode.auto.testing;

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
import org.firstinspires.ftc.teamcode.utils.AutoHandoff;
import org.firstinspires.ftc.teamcode.utils.PanelsGraphs;
import org.firstinspires.ftc.teamcode.subsystems.FlowerPickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * BiobuzzAuto: the ONE Auto that all 13 match Autos run. It is not on the Driver Station list by
 * itself. Each of the 13 small files (in auto/ and auto/testing/, for example AutoBlueD6ThreeVolley)
 * just picks:
 *   - ALLIANCE: BLUE or RED,
 *   - SPOT    : where the robot starts (see "THE 4 START SPOTS"),
 *   - ROUTINE : MOVE_PARK, SHOOT_PARK, THREE_VOLLEY or GARDEN_D1 (see "THE 4 ROUTINES").
 * Fix a bug HERE and all 13 Autos get the fix. (One small class that "extends" this one, no deeper:
 * 13 copies of 1000 lines would never stay the same.)
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
 * THE 4 ROUTINES
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
 *   GARDEN_D1    : (issue #156. The MAIN start spot only: tile D6 for blue. BLUE numbers, red is
 *                  turned 180, but only the blue Auto is on the Driver Station.)
 *     1. SHOT 1               : from the start spot, like SHOOT_PARK, but FASTER (see "FASTER SHOTS"
 *                               below). The flywheel is stopped when the shot ends.
 *     2. DRIVE_TO_GARDEN      : the robot ROTATES LEFT while it leaves the wall, then drives toward
 *                               the right side along the rear wall, facing right, with its LEFT
 *                               bumper running along the rear wall (GARDEN_WALL_GAP_INCHES away).
 *                               It stops GARDEN_APPROACH_X_INCHES from the left wall, just before the
 *                               line of POLLEN. (The coach's plan from the issue #156 comment.)
 *     3. GARDEN               : the intake starts a little before that stop. Then the robot SWEEPS
 *                               slowly down the line of POLLEN into the corner with the intake on
 *                               (the line is straight ahead of the intake), and sits there for
 *                               GARDEN_PICK_SECONDS. The flower bar stays UP the whole time.
 *     4. DRIVE_TO_SHOT_2      : AFTER LEAVING THE GARDEN the flywheel revs up to
 *                               GARDEN_LEAVE_FLYWHEEL_RPM. The robot goes down the right side of the
 *                               hive and under it to tile D1, turning left from facing right to
 *                               FACING UP (so it ends up backing down the field).
 *     5. SHOT 2               : from tile D1 (D1_SHOT_X/Y), facing up at the hive, also faster.
 *     6. DRIVE_TO_PARK        : straight to the loading zone, still facing up.
 *   If the robot is out of time after the garden, it skips the D1 drive and the shot, and drives
 *   straight from the garden to the loading zone (so it still parks).
 *   Estimated time: see README.md. The garden numbers are starting guesses.
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
 *   (GARDEN_D1 is different: it stops the flywheel after shot 1 and revs it again when it leaves
 *   the garden, see GARDEN_FLYWHEEL_OFF_AT_GARDEN.)
 *
 * ONE FLOWER (THREE_VOLLEY)
 *   On the way in, FLOWER_x_BAR_DOWN_INCHES before the stop, the bar goes DOWN and the intake starts.
 *   At the stop: wait until the bar is down, creep FLOWER_x_CREEP_INCHES straight in, sit there with
 *   the intake on for FLOWER_PICK_SECONDS, then stop the intake and drive on with the bar STILL DOWN.
 *   It goes back UP when the next shot starts feeding (or when that shot ends or is skipped).
 *
 * ONE GARDEN PICK (GARDEN_D1)
 *   The blue GARDEN is a strip in the top-right corner, along the rear wall. Its 4 POLLEN (2.8 inch
 *   balls) sit in a line from the corner: X about 130 to 141 on Pedro's map, touching the rear wall.
 *   Unlike a flower there is no bar: the balls are on the floor and the INTAKE takes them in. The
 *   robot faces right with its left bumper along the rear wall, so the line of balls is straight
 *   ahead of the intake and the robot takes them in one after another as it drives down the line.
 *   Like a flower: drive to a stop, move in (here: sweep along the wall), sit with the intake on,
 *   then drive on. The gate stays CLOSED, so the balls wait at the gate. The rules say a robot may
 *   hold no more than 4 balls (G407), so shot 1 has to empty the robot first.
 *
 * FASTER SHOTS (GARDEN_D1 only)
 *   The coach saw the 4th POLLEN leave late, after the robot had already started to move (issue
 *   #156 comment). So the GARDEN_D1 shots use GARDEN_FAST_FEED (the intake runs the whole feed
 *   instead of waiting for the flywheel to recover after each ball) and GARDEN_FEED_SECONDS. The
 *   robot does not start to drive until the shot (feed, then lift) is over. The other Autos still
 *   use FEED_SECONDS and INTAKE_WAITS_FOR_SPEED, so they shoot exactly as before.
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

    public enum Routine { MOVE_PARK, SHOOT_PARK, THREE_VOLLEY, GARDEN_D1 }

    private final VisionSubsystem.Alliance alliance;
    private final StartSpot startSpot;
    private final Routine routine;

    public BiobuzzAuto(VisionSubsystem.Alliance alliance, StartSpot startSpot, Routine routine) {
        // GARDEN_D1's paths all start from the MAIN spot (D6 for blue). From the FLOWER spot they
        // would drive straight through the hive, so stop right away with a clear message.
        if (routine == Routine.GARDEN_D1 && startSpot != StartSpot.MAIN) {
            throw new IllegalArgumentException("GARDEN_D1 can only start from StartSpot.MAIN");
        }
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

    // --- Garden (GARDEN_D1) ---
    // How long the robot sits in the corner with the intake on, after the sweep (seconds). The balls
    // are mostly taken in during the sweep, so this is only to catch the last one.
    // TODO: starting guess. Time how long the intake really needs to take in all the POLLEN.
    private static final double GARDEN_PICK_SECONDS = 1.0;
    // How far before the sweep starts (the approach stop) the intake turns on (inches).
    // TODO: starting guess. The intake only needs to be on before it reaches the first ball.
    private static final double GARDEN_INTAKE_ON_INCHES = 12.0;
    // The most time the sweep down the line of POLLEN may take before the pick starts anyway (it
    // may stop against the right wall). The sweep is a short drive, so the robot never gets fast.
    // TODO: starting guess. If the balls get PUSHED instead of taken in, the sweep is too fast:
    // we would need a speed limit for Pedro, which our code does not use anywhere yet (ask first).
    private static final double GARDEN_SWEEP_TIMEOUT_SECONDS = 3.0;

    // --- Faster shots (GARDEN_D1 only, issue #156 comment) ---
    // The coach saw the 4th POLLEN leave late, with the robot already starting to move.
    // true  = the intake runs the WHOLE feed, so the balls follow each other quickly.
    // false = like the other Autos (INTAKE_WAITS_FOR_SPEED): after each ball the intake waits for
    //         the flywheel to get back to speed, which is slower. Set it false if the flywheel
    //         dips so much that the later balls fall short.
    private static final boolean GARDEN_FAST_FEED = true;
    // How long the gate stays OPENED while the intake feeds, for the GARDEN_D1 shots (seconds).
    // The other Autos use FEED_SECONDS (3.0). The robot does not move until the feed AND the
    // lift (GATE_LIFT_SECONDS) are over.
    // TODO: starting guess. Watch the 4th ball: still late, raise it. All 4 out early, lower it.
    private static final double GARDEN_FEED_SECONDS = 2.5;

    // --- Flywheel during GARDEN_D1 (issue #156 comment) ---
    // true  = the flywheel stops when shot 1 ends, and revs up to GARDEN_LEAVE_FLYWHEEL_RPM as the
    //         robot LEAVES the garden, so it is close to shooting speed when it reaches D1.
    // false = like the other Autos: it keeps spinning at PRE_REV_FLYWHEEL_RPM the whole time.
    private static final boolean GARDEN_FLYWHEEL_OFF_AT_GARDEN = true;
    // The flywheel speed as the robot leaves the garden (the drive to D1). The shot at D1 then
    // takes over (PRE_REV_FLYWHEEL_RPM while it looks for the hive, then the shot table's RPM).
    // Only used when GARDEN_FLYWHEEL_OFF_AT_GARDEN is true. 1458 or lower.
    private static final double GARDEN_LEAVE_FLYWHEEL_RPM = 1200.0;

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

    // ---- GARDEN_D1 (issue #156) ----
    // THE GARDEN. The blue GARDEN is a strip in the TOP-RIGHT corner along the rear wall. Its 4 POLLEN
    // (2.8 inch balls) sit in a line from the corner: X about 130.3 to 141.5, touching the rear wall.
    // The robot faces RIGHT (heading 0, the intake leads), with its LEFT bumper running along the
    // rear wall, so the line of balls is straight ahead of the intake. It stops at the APPROACH
    // spot, just before the first ball, then SWEEPS down the line into the corner.
    //   Y: the rear wall is at 141.5 and the robot is 16.5 in wide, so its left side is
    //      GARDEN_WALL_GAP_INCHES from the wall. The robot is not exactly 16.5 in wide (issue #55)
    //      and the Pinpoint drifts, so a small gap means it may rub the wall. That is OK.
    //   Sweep end: the front of the robot ends GARDEN_CORNER_GAP_INCHES from the right wall.
    //   Approach X 118: the front of the robot is then about 3 in before the first ball (X 130.3).
    // TODO: starting guesses, from the manual's drawing, not measured. Check on the robot that the
    // intake takes in all 4 balls, the one in the very corner included. If the robot scrapes or
    // jams on the rear wall, raise GARDEN_WALL_GAP_INCHES. If it misses the balls on the wall
    // side, lower it.
    private static final double GARDEN_WALL_GAP_INCHES = 0.25;
    private static final double GARDEN_CORNER_GAP_INCHES = 0.6;
    private static final double GARDEN_APPROACH_X_INCHES = 118.0;
    private static final double GARDEN_HEADING_DEGREES = 0.0;
    private static final double GARDEN_Y_INCHES = FieldConstants.FIELD_SIZE_INCHES
            - FieldConstants.ROBOT_WIDTH_INCHES / 2.0 - GARDEN_WALL_GAP_INCHES;
    private static final double GARDEN_SWEEP_END_X_INCHES = FieldConstants.FIELD_SIZE_INCHES
            - FieldConstants.ROBOT_LENGTH_INCHES / 2.0 - GARDEN_CORNER_GAP_INCHES;

    // Start (D6) -> approach spot. The robot ROTATES LEFT (from facing down, to facing right: -90 ->
    // 0, counterclockwise) while it leaves the wall. It cannot turn in place: it starts with its
    // back on the wall, and its corners would hit the wall. So the path first dips down away from
    // the wall, turns there (between TURN_START and TURN_END, as fractions of the way along the
    // path), then rises back up to the wall and ends driving right, along it. These numbers keep the
    // robot's corners at least 9 in from the wall during the turn and 17 in from the hive (checked
    // by a geometry script, not on the robot).
    private static final double TO_GARDEN_CONTROL1_X = 100.0;
    private static final double TO_GARDEN_CONTROL1_Y = 104.0;
    private static final double TO_GARDEN_CONTROL2_X = 104.0;
    private static final double TO_GARDEN_CONTROL2_Y = 130.0;
    private static final double TO_GARDEN_TURN_START_FRACTION = 0.35;
    private static final double TO_GARDEN_TURN_END_FRACTION = 0.62;

    // The D1 SHOT spot (GARDEN_D1 shot 2): in tile D1, facing up at the hive, the same way the robot
    // faces when it starts from D1.
    //   X 80: the whole robot (16.5 in wide) stays inside column D (X 70.75 to 94.3), so it stays on
    //         our half of the field, and its right side is about 3 in from the audience-wall flower.
    //   Y 20: not jammed against the audience wall or the flower, like the D1 START spot is.
    // It is 51 in from the hive, a little closer than D6 (62 in). The camera sets the RPM from ty.
    // TODO: starting guess. Check on the robot that the camera sees the hive from here and that the
    // shot goes in. Move it back toward the wall (smaller Y) if the shot table says it is too close.
    private static final double D1_SHOT_X_INCHES = 80.0;
    private static final double D1_SHOT_Y_INCHES = 20.0;
    private static final double D1_SHOT_HEADING_DEGREES = 90.0;
    // Garden corner -> D1 shot spot: it leaves the corner sideways (the robot still faces right, so
    // it slides down along the right side of the hive), then goes left under the hive to the D1
    // spot. It turns LEFT from facing right to FACING UP (0 -> 90, counterclockwise) between
    // TURN_START and TURN_END, so it ends up backing down the field. The turn waits until the robot
    // is well clear of the rear wall. Checked by a geometry script to stay at least 4 in from the
    // hive and the flowers, and 13 in from the walls during the turn (not on the robot).
    private static final double GARDEN_TO_D1_CONTROL1_X = 104.0;
    private static final double GARDEN_TO_D1_CONTROL1_Y = 80.0;
    private static final double GARDEN_TO_D1_CONTROL2_X = 130.0;
    private static final double GARDEN_TO_D1_CONTROL2_Y = 38.0;
    private static final double GARDEN_TO_D1_TURN_START_FRACTION = 0.30;
    private static final double GARDEN_TO_D1_TURN_END_FRACTION = 0.80;
    // Garden corner -> park, used only when the robot is out of time and skips the D1 shot. The same
    // kind of path: leave the corner, turn left to face up, then go down to the loading zone. It stays
    // at least 7 in from the flower on the right wall (around Y 94) and the hive.
    private static final double GARDEN_TO_PARK_CONTROL1_X = 118.0;
    private static final double GARDEN_TO_PARK_CONTROL1_Y = 117.0;
    private static final double GARDEN_TO_PARK_CONTROL2_X = 110.0;
    private static final double GARDEN_TO_PARK_CONTROL2_Y = 55.0;
    private static final double GARDEN_TO_PARK_TURN_START_FRACTION = 0.18;
    private static final double GARDEN_TO_PARK_TURN_END_FRACTION = 0.36;

    // ==================================================================
    // The steps. The robot is always in exactly one of these.
    // ==================================================================
    // true once START was pressed. stop() only saves the ending heading for the TeleOp if the Auto
    // really ran (an Auto that was only INIT'd and then stopped must not hand anything over).
    private boolean autoStarted = false;

    private enum AutoState {
        SHOT_PAUSE, SHOT_FIND_HIVE, SHOT_WAIT_FOR_SPEED, SHOT_FEED_BALLS, SHOT_LIFT_LAST_BALL,
        FLOWER_BAR_DOWN, FLOWER_CREEP, FLOWER_PICK, FLOWER_BAR_UP,
        GARDEN_SWEEP, GARDEN_PICK,   // GARDEN_D1 only
        DRIVE_TO_FLOWER_A, DRIVE_TO_SHOT_2, DRIVE_TO_FLOWER_B, DRIVE_TO_SHOT_3, DRIVE_TO_GARDEN,
        DRIVE_TO_PARK,
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
    // Sends this Auto's numbers to Panels (practice graphs). The master switch is in PanelsGraphs.
    private PanelsGraphs panelsGraphs;

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
    // GARDEN_D1 only (these stay null for the other routines). For GARDEN_D1, shot2Pose (above) is
    // the D1 shot spot instead of the spot behind flower A.
    private Pose gardenPose;      // the approach spot, just before the line of POLLEN
    private Pose gardenEndPose;   // the end of the sweep, in the corner

    // Paths, built once in init().
    private Path startToParkPath;
    private Path toFlowerAPath;
    private Path flowerACreepPath;
    private Path toShot2Path;
    private Path toFlowerBPath;
    private Path flowerBCreepPath;
    private Path flowerBToShot3Path;
    private Path shot3ToParkPath;
    // GARDEN_D1 only (null for the other routines).
    private Path startToGardenPath;   // start -> the approach spot, turning left on the way
    private Path gardenSweepPath;     // the approach spot -> the corner, down the line of POLLEN
    private Path gardenToShot2Path;    // garden -> the D1 shot spot
    private Path gardenToParkPath;     // garden -> park, only when out of time
    private Path shot2ToParkPath;      // the D1 shot spot -> park

    private int lastShotNumber;   // 0, 1, 2 or 3, from the routine

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
    private String gardenResult = "-";
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
            if (routine == Routine.GARDEN_D1) {
                // GARDEN_D1 shoots from D1 facing up, so it parks facing up too (it never turns
                // around to park). Same spot as the other parks.
                parkPose = makePose(FieldConstants.BLUE_PARK_X_INCHES, FieldConstants.BLUE_PARK_Y_INCHES,
                        D1_SHOT_HEADING_DEGREES);
            } else {
                parkPose = FieldConstants.parkPose(alliance);
            }
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
        if (routine == Routine.GARDEN_D1) {
            // The approach spot (just before the line of POLLEN) and the end of the sweep (in the
            // corner). Both face right, with the left bumper along the rear wall.
            gardenPose = makePose(GARDEN_APPROACH_X_INCHES, GARDEN_Y_INCHES, GARDEN_HEADING_DEGREES);
            gardenEndPose = makePose(GARDEN_SWEEP_END_X_INCHES, GARDEN_Y_INCHES, GARDEN_HEADING_DEGREES);
            // Shot 2 is taken from tile D1, not from the spot behind flower A.
            shot2Pose = makePose(D1_SHOT_X_INCHES, D1_SHOT_Y_INCHES, D1_SHOT_HEADING_DEGREES);
        }

        if (routine == Routine.MOVE_PARK) {
            lastShotNumber = 0;
        } else if (routine == Routine.SHOOT_PARK) {
            lastShotNumber = 1;
        } else if (routine == Routine.GARDEN_D1) {
            lastShotNumber = 2;
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
        if (routine == Routine.GARDEN_D1) {
            gardenResult = "not yet";
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
        // Panels graphs (the master switch is inside PanelsGraphs). It only LOOKS, it moves nothing.
        panelsGraphs = new PanelsGraphs();
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
        // Open this loop's Panels frame. The matching send() is the LAST line of loop().
        panelsGraphs.beginLoop();

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
                if (flywheelReady || !isFeedPaced()) {
                    intakeSubsystem.intake();
                } else {
                    intakeSubsystem.stop();
                }
                if (feedWasReady && !flywheelReady) {
                    flywheelDipCount = flywheelDipCount + 1;   // about one per ball
                }
                feedWasReady = flywheelReady;
                if (stateTimer.seconds() >= feedSeconds()) {
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

            // ---------------- ONE GARDEN PICK (GARDEN_D1) ----------------
            case GARDEN_SWEEP:
                // Sweep down the line of POLLEN into the corner with the intake on, the left bumper
                // along the rear wall. No flower bar: the POLLEN are on the floor. (The bar stays UP
                // because of the goToUp() at the top of loop().)
                if (actionsOn) {
                    intakeSubsystem.intake();
                }
                if (!follower.isBusy() || legTimer.seconds() >= GARDEN_SWEEP_TIMEOUT_SECONDS) {
                    if (!HOLD_AT_STOPS) {
                        follower.stop();
                    }
                    stateTimer.reset();
                    autoState = AutoState.GARDEN_PICK;
                }
                break;

            case GARDEN_PICK:
                if (actionsOn) {
                    intakeSubsystem.intake();
                }
                double gardenPickSeconds = actionsOn ? GARDEN_PICK_SECONDS : INTAKE_PAUSE_SECONDS;
                if (stateTimer.seconds() >= gardenPickSeconds) {
                    gardenResult = actionsOn
                            ? String.format("picked for %.1f s", GARDEN_PICK_SECONDS)
                            : "pause (actions off)";
                    finishGarden();
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

            case DRIVE_TO_GARDEN:
                turnIntakeOnNearGarden();
                if (isDriveDone("garden")) {
                    finishDrive("Garden", gardenPose, autoState);
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

        // 4. Panels graphs: add this Auto's numbers, then send the frame (the LAST line of loop()).
        addPanelsGraphs();
        panelsGraphs.send();
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
            // Park from where the last shot was.
            startDrive(pathToParkAfterLastShot(), AutoState.DRIVE_TO_PARK);
        } else if (routine == Routine.GARDEN_D1) {
            // GARDEN_D1 has only one thing between its two shots: the garden.
            startDrive(startToGardenPath, AutoState.DRIVE_TO_GARDEN);
        } else if (shotNumber == 1) {
            startDrive(toFlowerAPath, AutoState.DRIVE_TO_FLOWER_A);
        } else {
            startDrive(toFlowerBPath, AutoState.DRIVE_TO_FLOWER_B);
        }
    }

    // The park drive that starts from where the last shot was taken: the D1 spot (GARDEN_D1), the
    // shot 3 spot, or the start (shot 1).
    private Path pathToParkAfterLastShot() {
        if (routine == Routine.GARDEN_D1) {
            return shot2ToParkPath;
        }
        return shotNumber == 3 ? shot3ToParkPath : startToParkPath;
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
            case DRIVE_TO_GARDEN:
                startGarden();
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
            if (shotNumber < lastShotNumber && !stopsFlywheelAfterThisShot()) {
                shooterSubsystem.setTargetRpm(PRE_REV_FLYWHEEL_RPM);   // keep it spinning
            } else {
                shooterSubsystem.stop();                               // last shot (or GARDEN_D1 shot 1)
            }
        }
        afterShot();
    }

    // GARDEN_D1 stops the flywheel after shot 1, and revs it again as the robot leaves the garden
    // (afterGarden). Every other routine keeps it spinning between shots, like before.
    private boolean stopsFlywheelAfterThisShot() {
        return routine == Routine.GARDEN_D1 && GARDEN_FLYWHEEL_OFF_AT_GARDEN;
    }

    // true = the intake only runs while the flywheel is at speed (balls go one at a time, slower).
    // GARDEN_D1 can turn that off (GARDEN_FAST_FEED) so its shots are faster.
    private boolean isFeedPaced() {
        if (routine == Routine.GARDEN_D1 && GARDEN_FAST_FEED) {
            return false;
        }
        return INTAKE_WAITS_FOR_SPEED;
    }

    // How long the gate stays OPENED while the intake feeds. GARDEN_D1 uses its own, shorter number.
    private double feedSeconds() {
        if (routine == Routine.GARDEN_D1) {
            return GARDEN_FEED_SECONDS;
        }
        return FEED_SECONDS;
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
    // The garden (GARDEN_D1)
    // ==================================================================
    // ACTIONS ON: while driving TO the garden, start the intake once the robot is within
    // GARDEN_INTAKE_ON_INCHES of the approach spot. The gate is already CLOSED from the shot, so the
    // balls wait at the gate. The flower bar is not used: the POLLEN are on the floor.
    private void turnIntakeOnNearGarden() {
        if (!actionsOn) {
            return;
        }
        if (follower.distanceToEndpoint() <= GARDEN_INTAKE_ON_INCHES) {
            shooterServoSubsystem.goToClosed();
            intakeSubsystem.intake();
        }
    }

    // The robot has stopped at the approach spot. Sweep down the line of POLLEN, unless there is no
    // time left.
    private void startGarden() {
        stateTimer.reset();
        if (actionsOn && isOutOfTime()) {
            gardenResult = "SKIPPED: out of time";
            afterGarden();
            return;
        }
        if (actionsOn) {
            shooterServoSubsystem.goToClosed();   // the balls wait at the gate
        }
        legTimer.reset();
        follower.follow(gardenSweepPath);
        autoState = AutoState.GARDEN_SWEEP;
    }

    private void finishGarden() {
        if (actionsOn) {
            intakeSubsystem.stop();
        }
        afterGarden();
    }

    // After the garden: drive to the D1 shot. If there is no time left for that, skip it and go
    // straight to the loading zone, so the robot still parks (8 points beat a missed shot).
    private void afterGarden() {
        if (actionsOn && isOutOfTime()) {
            shotResults[1] = "SKIPPED: out of time, went straight to park";
            startDrive(gardenToParkPath, AutoState.DRIVE_TO_PARK);
        } else {
            if (actionsOn && GARDEN_FLYWHEEL_OFF_AT_GARDEN) {
                // The robot is leaving the garden: rev the flywheel up now, so it is already close
                // to shooting speed when the robot gets to D1.
                shooterSubsystem.setTargetRpm(GARDEN_LEAVE_FLYWHEEL_RPM);
            }
            startDrive(gardenToShot2Path, AutoState.DRIVE_TO_SHOT_2);
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
            case GARDEN_SWEEP:
            case GARDEN_PICK:
                // No bar to bring up here, so stop the sweep and go. finishGarden() sees there is
                // no time left and drives straight to park.
                gardenResult = "CUT SHORT: out of time";
                if (!HOLD_AT_STOPS) {
                    follower.stop();
                }
                finishGarden();
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

        if (routine == Routine.GARDEN_D1) {
            buildGardenPaths();
        }
    }

    // The GARDEN_D1 paths. Built only for that routine, so the other 12 Autos never touch them.
    private void buildGardenPaths() {
        // A "piecewise" heading is one rule until a point on the path, then another rule. The points
        // are fractions of the way along the path (0 = the start, 1.0 = the end). Every turn here is
        // a quarter turn to the LEFT (counterclockwise), so there is no doubt which way Pedro turns.

        // Start (D6, facing down) -> the approach spot (facing right, left bumper by the rear wall).
        // The robot ROTATES LEFT while it leaves the wall:
        //   until TURN_START : keep facing down while the robot backs away from the wall,
        //   until TURN_END   : turn from facing down to facing right,
        //   until 1.0        : keep facing right while it drives along the wall to the approach spot.
        Interpolator turnLeftOnTheWay = Interpolator.piecewise()
                .until(TO_GARDEN_TURN_START_FRACTION, Interpolator.constant(startPose))
                .until(TO_GARDEN_TURN_END_FRACTION, Interpolator.linear(startPose, gardenPose))
                .until(1.0, Interpolator.constant(gardenPose));
        startToGardenPath = Paths.curve(startPose,
                makePoint(TO_GARDEN_CONTROL1_X, TO_GARDEN_CONTROL1_Y),
                makePoint(TO_GARDEN_CONTROL2_X, TO_GARDEN_CONTROL2_Y),
                gardenPose).heading(turnLeftOnTheWay);

        // The sweep: a short straight line down the line of POLLEN into the corner, still facing
        // right. It is short on purpose, so the robot never gets fast on it.
        gardenSweepPath = Paths.line(gardenPose, gardenEndPose).constant(gardenPose);

        // Garden corner -> D1 shot spot. The robot leaves the corner facing right, then turns LEFT
        // to face up (0 -> 90), so it ends up backing down the field to the D1 spot.
        Interpolator turnToFaceUpForD1 = Interpolator.piecewise()
                .until(GARDEN_TO_D1_TURN_START_FRACTION, Interpolator.constant(gardenEndPose))
                .until(GARDEN_TO_D1_TURN_END_FRACTION, Interpolator.linear(gardenEndPose, shot2Pose))
                .until(1.0, Interpolator.constant(shot2Pose));
        gardenToShot2Path = Paths.curve(gardenEndPose,
                makePoint(GARDEN_TO_D1_CONTROL1_X, GARDEN_TO_D1_CONTROL1_Y),
                makePoint(GARDEN_TO_D1_CONTROL2_X, GARDEN_TO_D1_CONTROL2_Y),
                shot2Pose).heading(turnToFaceUpForD1);

        // Garden corner -> park, only used when the robot is out of time. The same kind of turn: it
        // leaves the corner facing right, turns left to face up, then goes down to the loading zone.
        Interpolator turnToFaceUpForPark = Interpolator.piecewise()
                .until(GARDEN_TO_PARK_TURN_START_FRACTION, Interpolator.constant(gardenEndPose))
                .until(GARDEN_TO_PARK_TURN_END_FRACTION, Interpolator.linear(gardenEndPose, parkPose))
                .until(1.0, Interpolator.constant(parkPose));
        gardenToParkPath = Paths.curve(gardenEndPose,
                makePoint(GARDEN_TO_PARK_CONTROL1_X, GARDEN_TO_PARK_CONTROL1_Y),
                makePoint(GARDEN_TO_PARK_CONTROL2_X, GARDEN_TO_PARK_CONTROL2_Y),
                parkPose).heading(turnToFaceUpForPark);

        // D1 shot spot -> park: a short straight line, still facing up.
        shot2ToParkPath = Paths.line(shot2Pose, parkPose).constant(parkPose);
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
        if (routine == Routine.GARDEN_D1) {
            telemetry.addData("Garden", gardenResult);
            telemetry.addData("Shot 2 (D1)", shotResults[1]);
        }
        telemetry.addData("Drives that timed out", legTimeouts);
    }

    private void addPoseTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pose X / Y / heading", "%.1f / %.1f / %.0f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    // ------------------------------------------------------------------
    // PANELS GRAPH NUMBERS (issue [#138]). Plain numbers with the units in the names (see
    // PanelsGraphs). Each call returns at once on a loop Panels will not send. There are no
    // flywheel amps here on purpose: asking the hub for the current costs loop time, and Pedro
    // wants a fast loop.
    // ------------------------------------------------------------------
    private void addPanelsGraphs() {
        // Where Pedro says the robot is. Pedro's heading is in radians, so turn it into degrees.
        Pose pose = follower.pose();
        panelsGraphs.addPose(pose.x(), pose.y(), Math.toDegrees(pose.heading()));

        // How far the robot still has to go on the path it is following. Only asked while Pedro is
        // really following a path (there is no "end of the path" when it is not).
        if (follower.isBusy()) {
            panelsGraphs.add("Distance to end of path (in)", follower.distanceToEndpoint());
        }

        // Where we are in the routine. The step number is the position in the AutoState list near
        // the top of this file (0 = SHOT_PAUSE, and DONE is last).
        panelsGraphs.add("Auto step number", autoState.ordinal());
        panelsGraphs.addText("Auto step", autoState.toString());
        panelsGraphs.add("Shot number", shotNumber);
        panelsGraphs.add("Run time (s)", robotSeconds());
        panelsGraphs.add("Time in this step (s)", stateTimer.seconds());

        // The shot numbers, only for routines that shoot.
        if (actionsOn && routine != Routine.MOVE_PARK) {
            panelsGraphs.addShooter(shooterSubsystem, false);
            panelsGraphs.addTurret(turretSubsystem);
            panelsGraphs.addVision(visionSubsystem);
            panelsGraphs.addGate(shooterServoSubsystem);
            panelsGraphs.addIntake(intakeSubsystem);
            panelsGraphs.addFlag("Flywheel ready (1=yes)", flywheelReady);
        }
    }
}
