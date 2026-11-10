package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HiveBearingMemory;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * TeleOp Alpha 11
 * An iterative OpMode with TWO controllers:
 *   gamepad1 = the DRIVER: drives the robot, runs the INTAKE, and holds the trigger to SHOOT.
 *              (The turret AUTO-TRACKS the hive ALL THE TIME, see ALWAYS-TRACK below.)
 *   gamepad2 = the OPERATOR: sets the shooter speeds (RPM), moves the GATE, and can run the
 *              INTAKE too. The operator does NOT aim the turret by hand any more.
 * It is DriveAlpha, IntakeAlpha, ShooterRPMAlpha, TurretAlpha, VisionAlpha, ShotTableAlpha and
 * PinpointAlpha put together, so each part has already been tested on its own. This OpMode owns
 * NO hardware. It only reads the gamepads and tells the subsystems what to do.
 *
 * CONTROL CHANGES (issue #74). The screen name stays "TeleOp - Alpha 11" because the Autos'
 * preselectTeleOp uses that name.
 *   REMOVED from gamepad2: the turret stick, the turret presets (Y, X, B), the Right Stick Button
 *     ALWAYS-TRACK switch, the gate triggers, and the shooter-wheel reverse bumpers.
 *   REMOVED from gamepad1: the X / B / Y gate buttons (they moved to gamepad2).
 *   MOVED: the gate is now gamepad2 X = CLOSED, B = OPENED, Y = LIFTED.
 *   NEW on gamepad2: Right Bumper = INTAKE and Left Bumper = REVERSE, the same as gamepad1.
 *   SWAPPED on gamepad2: Start = alliance (was Back), Back = flywheel speed DIAL / TABLE (was Start).
 *   CHANGED: gamepad2 A is now HOLD to float the turret (so you can turn it by hand). Letting go
 *     goes back to tracking. (A used to switch ALWAYS-TRACK off, and nothing is left to turn it on.)
 *   NOTE: there is NO way to reverse the shooter wheels now, so a jam in the wheels cannot be
 *     cleared from the controllers. (The intake REVERSE still works.)
 *   The full button map for both gamepads is in TeamCode/docs/biobuzz-teleop-controls.md.
 *
 * NEW IN ALPHA 11: FIELD-CENTRIC DRIVE (issue #54)
 *   The drive now STARTS FIELD-CENTRIC: pushing the left stick forward moves the robot AWAY FROM THE
 *   DRIVER, whichever way the robot is turned. (Before, "forward" always meant the way the robot
 *   faces. That is still there: ROBOT-CENTRIC.) It uses DriveSubsystem.driveFieldCentric() and the
 *   Pinpoint's heading.
 *   - THE OPERATOR switches the drive FIELD-CENTRIC <-> ROBOT-CENTRIC with the MODE button. Which
 *     button that is on our controller is still being found out. The INIT screen shows two
 *     candidates, "guide" (the center Logitech logo) and "left stick button". Press them and see
 *     which one says true. MODE_BUTTON_GUIDE_ENABLED and MODE_BUTTON_LEFT_STICK_ENABLED turn each
 *     one on or off. The drive mode is on the screen next to the alliance line, big.
 *     (The left stick is not used for the turret any more, so the Left Stick Button is no longer
 *     under a stick that gets pushed hard.)
 *   - THE DRIVER's D-pad Left RE-AIMS: "the way the robot faces right now is away from me". It only
 *     changes which way forward means. The Pinpoint is NEVER reset, because the turret's hive
 *     memory depends on its heading and position.
 *   - WHICH WAY IS "AWAY FROM THE DRIVER" depends on the alliance. From the field picture the BLUE
 *     alliance area is on the RIGHT and the RED one on the LEFT, so the blue driver faces LEFT (field
 *     heading 180) and the red driver faces RIGHT (0). The Pinpoint's heading is zero wherever the
 *     robot sits at INIT, and our Autos never turn the robot, so after Auto it faces the way it
 *     started: BLUE facing DOWN (-90), RED facing UP (+90). The heading that goes into the drive is
 *     the Pinpoint's heading plus (the robot's field heading at INIT - the driver's forward field
 *     heading). Blue: -90 - 180 = +90. Red: +90 - 0 = +90. The same answer, because red is the blue
 *     plan turned 180 degrees. The four numbers are kept per alliance (see the constants), so a
 *     different start orientation later is a one-number change. The offset is set from the
 *     alliance (operator Start) in INIT and fixed at START: changing the alliance after START does
 *     NOT change which way the stick drives. D-pad Left replaces it with the re-aim.
 *   - ASSUMPTIONS: the blue drivers stand on the right and the red drivers on the left, and the robot
 *     is not moved between the end of Auto and INIT (the Pinpoint is zeroed at INIT). With no Auto
 *     before (practice), put the robot in the Auto start orientation, or press D-pad Left.
 *   - THE SAFETY RULE: if the Pinpoint is not READY after START, or its heading jumps (the same
 *     check that protects the hive memory), field-centric switches ITSELF OFF and the robot drives
 *     ROBOT-CENTRIC. The screen says why. It does NOT switch itself back on, because a restarted
 *     Pinpoint has a new zero. The operator's MODE button or the driver's D-pad Left turns it on again.
 *   - The turret still follows the robot's spin the same way (the turn stick is the same in both
 *     modes), and the hive memory is not touched by any of this.
 *
 * NEW IN ALPHA 10: SAFER PINPOINT HEADING FOLLOW
 *   (after the first robot test, where the turret snapped around):
 *      - The remembered hive direction is THROWN AWAY whenever the Pinpoint is not READY, or its
 *        heading jumps by more than MAX_PINPOINT_HEADING_JUMP_DEGREES in one loop (a robot cannot
 *        turn that fast, so the Pinpoint must have restarted, for example after a power dip).
 *        Before, a restarted Pinpoint (heading back at 0) could send the turret to a wrong angle.
 *      - If the remembered hive is somewhere the turret cannot go (for example behind the robot),
 *        the turret now HOLDS where it is. Before, a small turn could flip the target between the
 *        two limits and swing the turret all the way across.
 *
 * NEW IN ALPHA 9: THE PINPOINT HELPS THE TURRET (and our hive's two sides)
 *   1. PINPOINT HEADING FOLLOW. While the camera sees the hive, HiveBearingMemory saves which
 *      direction the hive is in on the FIELD: robotHeading + turretAngle - tx. If the camera then
 *      loses the hive (the arm hides the tags, a ball blocks the view), ALWAYS-TRACK keeps the
 *      turret aimed at that remembered direction: turret angle = remembered direction - robotHeading.
 *      So the turret keeps pointing at the hive while the robot SPINS. It gives up (STALE, the
 *      turret holds still) when the Pinpoint says the robot has DRIVEN more than 12 inches since the
 *      camera last saw the hive, or after 5 seconds, because driving changes the real direction
 *      and heading alone cannot see that. When the camera sees the hive again it takes over.
 *      If the Pinpoint is missing or not READY, the turret just holds, like Alpha 8. It does not
 *      search. The screen line "Hive memory" says FOLLOWING, STALE or none.
 *   2. BOTH SIDES OF OUR HIVE. Each hive has two cells (FAR and AUDIENCE), and only the raised one
 *      scores. VisionSubsystem now LOCKS onto one cell, switches to our other cell only when it is
 *      clearly higher, and lets go of the lock after the cell has been out of view for half a
 *      second. After a hive tip you drive to the other side, the old cell leaves the view, and the
 *      cell you now see becomes the target by itself. The screen says "Aiming at BLUE FAR side"
 *      (or AUDIENCE). There is NO new button. The alliance button (gamepad2 Start) also clears
 *      the memory and the lock, so we never aim at the other hive's direction.
 *   LIMITS: no search (if the camera has not seen the hive from the new side, turn the robot
 *   toward the hive until it does). And if only the LOWERED cell is in view, ty alone cannot say
 *   it is lowered, so the turret would still aim at it.
 *
 * From ALPHA 8: ODOMETRY ON THE SCREEN
 *   PinpointSubsystem reads the goBILDA Pinpoint odometry computer once per loop, and the Driver
 *   Station shows where the robot is: X (inches forward), Y (inches left), and heading (degrees,
 *   turning left is positive), plus the speeds and the Pinpoint's status. The numbers are shown
 *   during INIT too, so check the status says READY before START. The robot must be STILL during
 *   INIT, because the Pinpoint calibrates its gyro then, and X, Y and heading start at 0 where the
 *   robot sits at INIT. As of Alpha 9 the turret uses the
 *   heading and position (see above). If the config has no device named "pinpoint", the screen
 *   says so and everything else still works.
 *
 * GAMEPAD 2 (the operator) has NO layers. Every button always does the same thing:
 *   X -> shooter GATE CLOSED (holds the ball back)
 *   B -> shooter GATE OPENED (lets the ball through)
 *   Y -> shooter GATE LIFTED (pushes the last ball in)
 *      (One move per press. The gate positions are CLOSED_POSITION, OPENED_POSITION and
 *       LIFTED_POSITION in ShooterServoSubsystem. If more than one is pressed on the same loop,
 *       CLOSED wins, then OPENED, then LIFTED.)
 *   A -> HOLD to STOP the turret (it floats, so you can turn it by hand). Let go and it goes
 *        back to tracking the hive.
 *   D-pad Up / Down  -> flywheel target RPM +/- FLYWHEEL_RPM_STEP
 *   D-pad Right / Left -> ratio (hood wheel / flywheel) +/- 0.05
 *   Right Bumper     -> INTAKE (pull balls in)       Left Bumper -> REVERSE (push balls out)
 *                       Same as the driver's bumpers. Left Bumper beats Right Bumper.
 *   Start            -> switch alliance BLUE <-> RED (starts as BLUE). Check it on the screen!
 *   Back             -> switch the flywheel speed source DIAL <-> TABLE (starts as DIAL)
 *   MODE button      -> switch the DRIVE FIELD-CENTRIC <-> ROBOT-CENTRIC (starts FIELD-CENTRIC). If
 *                       field-centric was switched off by the safety rule, it turns it back ON.
 *                       (The center "guide" button or the Left Stick Button, see NEW IN ALPHA 11.)
 *   Left Stick, Right Stick, both Triggers: not used.
 *   The turret has no other operator control. It tracks the hive all the time (see ALWAYS-TRACK
 *   below). It will not go past +/-120 degrees (see TurretSubsystem). 0 degrees is where
 *   the turret pointed when the Control Hub powered on, because the home switch is not
 *   installed yet, so power on with the turret pointing straight ahead.
 *
 * It starts FIELD-CENTRIC: "forward" means AWAY FROM THE DRIVER (see NEW IN ALPHA 11). ROBOT-CENTRIC
 * ("forward" means the way the robot is facing) is one MODE press away.
 *
 * GAMEPAD 1 - DRIVER
 *   Left Stick   -> drive forward / backward and slide (strafe) left / right
 *   Right Stick  -> turn (push right to turn right)
 *   HOLD the Left Trigger -> SLOW MODE for careful lining up
 *   D-pad Left   -> RE-AIM: "the way the robot faces right now is away from me" (field-centric).
 *      It does not touch the Pinpoint. It also turns field-centric back on after the safety rule.
 *   Right Bumper -> INTAKE (pull balls in)      Left Bumper -> REVERSE (push balls out)
 *      (Left Bumper beats Right Bumper, so clearing a jam always works. The operator's bumpers
 *       do the same, and REVERSE on EITHER gamepad beats INTAKE on either. The intake runs at the
 *       power set in IntakeSubsystem, DEFAULT_INTAKE_POWER.)
 *   Right Trigger -> HOLD to SHOOT. (The turret is already tracking the hive, see ALWAYS-TRACK.)
 *      The wheels run at the flywheel speed from the DIAL or the TABLE (see THE SHOT TABLE
 *      below), and the hood wheel at that x the ratio dial. At the start that is DIAL mode,
 *      flywheel 1300 RPM and hood wheel 2600 RPM (a ratio of 2.00).
 *      Let go and both wheels go to 0 power and coast down.
 *      The operator's A (hold) floats the turret, and that still beats the tracking.
 *   X / B / Y    -> not used. (The shooter gate moved to the operator's X / B / Y.)
 *
 * GAMEPAD 2 - OPERATOR, SHOOTER DIALS (SPEED CONTROL in RPM, read from the wheel encoders)
 *   D-pad Up / Down    -> flywheel target RPM UP / DOWN by FLYWHEEL_RPM_STEP
 *   D-pad Right / Left -> ratio UP / DOWN by 0.05
 *   The hood wheel target RPM is flywheel RPM x ratio (RATIO = HOOD WHEEL / FLYWHEEL). The
 *   shooter will not go past each wheel's own maximum (the flywheel motor is the 1620 RPM one,
 *   so its maximum is only 1458 RPM). If a wheel would be over its maximum, the screen says
 *   CAPPED and the real ratio is lower than the dial says.
 *   (The shooter wheels can no longer be reversed from this OpMode. ShooterRPMAlpha and
 *   ShooterPowerAlpha can still run them backward, for testing.)
 *   The screen shows "OK to shoot" only when BOTH wheels are really at their target RPM
 *   (the shooter's isAtSpeed()). It drops back to WAIT after a ball slows the wheels.
 *   ShooterRPMAlpha has the same wheel speed control on its own, for testing.
 *
 * THE SHOT TABLE (TABLE mode, switched with gamepad2 Back)
 *   The Limelight is on the turret. It reads ty, how far UP the hive's AprilTags appear in its
 *   view, and ShotTable turns that into a flywheel RPM. (Closer = bigger ty = lower RPM.)
 *   - DIAL mode: the flywheel uses the operator's dial, exactly like before. This is the safe
 *     fallback that needs no camera.
 *   - TABLE mode: while the camera sees the hive, the flywheel uses the table's RPM. If the
 *     camera loses the hive, it keeps the LAST table RPM for TABLE_RPM_HOLD_SECONDS so the wheels
 *     do not lurch, then goes back to the dial. The screen says which one is being used.
 *   - The camera can only see the hive when the turret points at it. ALWAYS-TRACK (below) keeps
 *     the turret pointed at it.
 *   - AUTO-ALIGN: the camera is ON the turret, so tx is how far the turret still has to turn.
 *     tx is positive to the RIGHT but turret angles are positive to the LEFT, so the new turret
 *     target is the turret angle NOW minus tx. If the camera loses the hive, the turret holds
 *     where it is. It does not search.
 *   - FOLLOWING THE ROBOT'S SPIN: auto-align alone only pushes once the turret is already off
 *     target, so when the driver turns the robot the turret always trails it. So while it aligns,
 *     the turret also gets a push the opposite way to the driver's turn (TURRET_FEEDFORWARD_GAIN).
 *     The screen shows it as "Turret follows spin". Tune the gain on the robot.
 *   - The table was measured aiming at one hive from a few spots, see ShotTable. Outside those
 *     spots it uses the nearest row, and the screen warns you.
 *
 * ALWAYS-TRACK (new in Alpha 7, and since issue #74 it is ALWAYS ON: there is no switch)
 *   The turret aligns on the hive ALL THE TIME, the whole match, with no button held. It uses
 *   the AUTO-ALIGN above, including following the robot's spin.
 *   - It does NOT run during INIT, because the robot must not move before START.
 *   - If the camera cannot see the hive, the turret HOLDS where it is. It does not search.
 *   - The operator's A (hold) floats the turret so it can be turned by hand. Let go of A and it
 *     goes back to tracking.
 *   - The ALLIANCE (gamepad2 Start) must be right, or the turret tracks the OTHER team's hive.
 *     It starts as BLUE. Check the alliance on the screen before you press START.
 *
 * THE INTAKE ALSO RUNS THE TRANSFER. One motor spins the front intake AND a hex shaft with
 * grippers in the transfer chute, so balls are carried up the chute while you drive.
 *
 * NOT IN THIS OPMODE: the one-wheel direction test. Use DriveAlpha for that, with the
 * robot on blocks.
 *
 * SAFE USE: keep hands, hair and loose clothing away from the shooter wheels, and keep
 * everyone out of the path of the ball.
 *
 * WHY THE STICK SIGNS ARE FLIPPED
 *   The drive subsystem uses the same convention as Pedro Pathing, Pinpoint and the turret:
 *   forward +, strafe LEFT +, turn COUNTERCLOCKWISE +. The controller's sticks are different:
 *   pushing a stick forward gives a NEGATIVE number, and pushing it right gives a POSITIVE
 *   number. So we flip the sign of all three.
 *
 * On the Driver Station, pair the controllers with Start + A (gamepad1) and Start + B (gamepad2).
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle, bumpers = L1 / R1,
 * triggers = L2 / R2.
 *
 * Robot configuration (on the Driver Station). The names live in the subsystem classes:
 *   Motor names: left_front, left_rear, right_front, right_rear   (DriveSubsystem)
 *   Motor name:  intake                                           (IntakeSubsystem)
 *   Motor names: flywheel, hood_wheel                             (ShooterSubsystem)
 *   Servo name:  shooter_servo   (ShooterServoSubsystem, goBILDA dual mode, POSITIONAL mode)
 *   Motor name:  turret_motor    (TurretSubsystem, with its encoder cable in the same port)
 *   Ethernet Device name: limelight   (VisionSubsystem, Limelight 3A. This robot's config MUST
 *                                      have it, or this OpMode stops at INIT.)
 *   I2C Device name: pinpoint         (PinpointSubsystem, goBILDA Pinpoint Odometry Computer. If it
 *                                      is missing, the screen says so and the OpMode still runs.)
 */
@TeleOp(name = "TeleOp - Alpha 11", group = "Testing")
public class BiobuzzTeleOp extends OpMode {

    // ------------------------------------------------------------------
    // Driving
    // ------------------------------------------------------------------
    // The sticks do not always rest at exactly zero. Anything smaller than this is ignored,
    // so the robot does not creep when you let go.
    private static final double STICK_DEADBAND = 0.05;

    // The trigger gives a number from 0.0 to 1.0. Past this much counts as "pressed".
    private static final double TRIGGER_PRESSED_THRESHOLD = 0.5;

    // ------------------------------------------------------------------
    // Field-centric drive (issue #54)
    // ------------------------------------------------------------------
    // WHICH BUTTON IS "MODE"? On the operator's controller (gamepad2) we are still finding out which
    // button reaches the Robot Controller. The INIT screen shows both, so press them and look. Then
    // set the one you do NOT want to false. (The Logitech F310's own MODE button may not be sent to
    // the Robot Controller in X mode. The center Logitech logo button is "guide". The Left Stick
    // Button is free now that the left stick no longer turns the turret.)
    private static final boolean MODE_BUTTON_GUIDE_ENABLED = true;
    private static final boolean MODE_BUTTON_LEFT_STICK_ENABLED = true;

    // The field headings that decide which way "away from the driver" is. 0 degrees is straight
    // toward the right wall, 90 is up the field, and turning left is positive (the same as Pedro
    // and the Pinpoint). The BLUE alliance area is on the RIGHT, so the blue driver faces LEFT (180).
    // The RED alliance area is on the LEFT, so the red driver faces RIGHT (0).
    private static final double DRIVER_FORWARD_FIELD_HEADING_BLUE_DEGREES = 180.0;
    private static final double DRIVER_FORWARD_FIELD_HEADING_RED_DEGREES = 0.0;

    // Which way the robot faces on the field at INIT, which is when the Pinpoint's heading is
    // zeroed. Our Autos never turn the robot, so after Auto it faces the way it started: blue
    // facing DOWN the field (-90), red facing UP (+90). (See FieldConstants in pedro/.)
    private static final double ROBOT_FIELD_HEADING_AT_INIT_BLUE_DEGREES = -90.0;
    private static final double ROBOT_FIELD_HEADING_AT_INIT_RED_DEGREES = 90.0;

    // ------------------------------------------------------------------
    // Shooter dials (gamepad2). These are test-only numbers.
    // ------------------------------------------------------------------
    // The flywheel dial is counted in WHOLE RPM (1300 means 1300 RPM).
    // Where it starts, how far one press moves it, and how low and high it can go.
    // It starts at 1300 RPM and the ratio starts at 2.00, so the hood wheel starts at 2600 RPM.
    // 1300 is the speed that looked about right on the robot. It is also the speed the shooter
    // falls back to in TABLE mode when the camera has not seen the hive lately.
    // The flywheel motor is now the goBILDA 1620 RPM one, so the dial stops at 1450, just under
    // the shooter's own limit (FLYWHEEL_MAX_TARGET_RPM, 1458 in ShooterSubsystem). It used to go
    // up to 3600 when the flywheel was a 6000 RPM motor.
    private static final int FLYWHEEL_RPM_START = 1300;
    private static final int FLYWHEEL_RPM_STEP = 100;
    private static final int FLYWHEEL_RPM_MIN = 500;
    private static final int FLYWHEEL_RPM_MAX = 1450;

    // The ratio dial is counted in HUNDREDTHS so that adding 5 over and over never drifts to
    // something like 1.9999999. So 200 means a ratio of 2.00, and a step of 5 means 0.05.
    private static final int RATIO_HUNDREDTHS_START = 200;
    private static final int RATIO_HUNDREDTHS_STEP = 5;
    private static final int RATIO_HUNDREDTHS_MIN = 100;
    private static final int RATIO_HUNDREDTHS_MAX = 300;

    // How many hundredths make a whole number (200 hundredths = 2.00).
    private static final double HUNDREDTHS_PER_ONE = 100.0;

    // A hood wheel target counts as CAPPED when it is higher than the shooter's maximum by more
    // than this many RPM. (A little room, so a tiny rounding error does not cry wolf.)
    private static final double CAPPED_TOLERANCE_RPM = 1.0;

    // The measured ratio (hood RPM / flywheel RPM) is only worth showing when the flywheel is
    // spinning faster than this. Below it the division gives silly numbers.
    private static final double MIN_RPM_FOR_MEASURED_RATIO = 300.0;

    // (The turret has no operator controls any more except A to float it, so there are no
    // turret power, stick or preset numbers here. The shooter wheels cannot be reversed from
    // this OpMode now either, so there are no reverse power numbers.)

    // ------------------------------------------------------------------
    // Vision and the shot table. These are test-only numbers, the same as in VisionAlpha.
    // ------------------------------------------------------------------
    // Auto-align picks a new turret target only when the camera's tx is bigger than this many
    // degrees. Smaller = more precise but twitchier. The hive window forgives about 5 degrees
    // or more, so there is no need to chase every fraction of a degree.
    private static final double ALIGN_REAIM_THRESHOLD_DEGREES = 1.0;

    // The Pinpoint's heading should never change by more than this many degrees from one loop to
    // the next (a loop is about 20 milliseconds, and 45 degrees in that time would be over 2000
    // degrees per second, far faster than the robot can turn). A bigger jump means the Pinpoint
    // restarted or glitched, so the remembered hive direction can no longer be trusted.
    private static final double MAX_PINPOINT_HEADING_JUMP_DEGREES = 45.0;

    // The target counts as "aimed at" when tx is within this many degrees of zero.
    private static final double AIMED_TOLERANCE_DEGREES = 2.0;

    // In TABLE mode, when the camera loses the hive, keep using the LAST table RPM for this many
    // seconds. After that the flywheel goes back to the dial. A short hold stops the wheels from
    // lurching if a tag drops out for a moment, and a limit stops a stale RPM from lasting.
    private static final double TABLE_RPM_HOLD_SECONDS = 1.0;

    // How much the turret FOLLOWS the robot's spin while it is auto-aligning.
    // On its own, auto-align only pushes once the turret is already off target, so a spinning
    // robot always gets ahead of it. This adds a push the OPPOSITE way to the driver's turn, so
    // the turret starts following right away. The push is: -(the robot's turn power) x this number.
    //   0.0  -> no following (auto-align by itself, like before).
    //   Higher -> the turret follows harder.
    // The right number is about (how fast the robot spins at full turn) / (how fast the turret
    // can turn at full power), which we have not measured. 0.30 is a SAFE starting guess that
    // follows a little less than needed, and auto-align cleans up the rest.
    // TO TUNE: auto-align on the hive, then spin the robot at a steady rate and watch tx.
    //   tx drifts to the same side the robot is spinning -> the turret is lagging, raise this.
    //   tx swings to the OTHER side -> the turret is leading, lower this.
    // Change it by about 0.05 at a time. Try slow mode too, it is scaled to match.
    private static final double TURRET_FEEDFORWARD_GAIN = 0.30;

    // true = the control map stays on the Driver Station screen while the OpMode RUNS (at the
    // bottom, under the live data). false = it is only shown before START, like it used to be.
    private static final boolean SHOW_CONTROLS_WHILE_RUNNING = true;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // LOOP TIME: how long one trip through loop() takes, in milliseconds. We time from the start
    // of one loop() to the start of the next, so it includes everything: the hub reads, the
    // subsystems, and the telemetry. A shorter loop means the turret, the shooter's READY check,
    // and the drive all update more often. This is only for measuring (to see whether
    // anything we add later, like bulk reads, really helps).
    private ElapsedTime loopTimer = new ElapsedTime();
    private int loopCount = 0;
    private double totalLoopMs = 0.0;
    private double worstLoopMs = 0.0;
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private ShooterServoSubsystem shooterServoSubsystem;
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;
    private PinpointSubsystem pinpointSubsystem;
    private HiveBearingMemory hiveMemory;
    private ShotTable shotTable;

    // The shooter dials, in whole numbers.
    private int flywheelRpmDial = FLYWHEEL_RPM_START;
    private int ratioHundredths = RATIO_HUNDREDTHS_START;

    // Where the flywheel speed comes from: false = the DIAL, true = the shot TABLE.
    // It starts on the DIAL, the fallback that needs no camera.
    private boolean tableMode = false;

    // The last RPM the table gave, and whether it has given one yet. timeSinceTarget counts the
    // seconds since the camera last saw the hive (it is reset every loop the hive is in view).
    private double lastTableRpm = 0.0;
    private boolean haveTableRpm = false;
    private ElapsedTime timeSinceTarget = new ElapsedTime();

    // What the buttons were doing LAST loop. A press lasts many loops (about 50 per second),
    // but we want it to count only ONCE. So we only act when it was NOT pressed last loop
    // and IS pressed now.
    private boolean wasGateClosedPressed = false;
    private boolean wasGateOpenedPressed = false;
    private boolean wasGateLiftedPressed = false;
    private boolean wasDpadLeftPressed = false;
    private boolean wasDpadRightPressed = false;
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;
    private boolean wasStartPressed = false;
    private boolean wasBackPressed = false;

    // The push the turret got this loop to follow the robot's spin, for the screen.
    private double turretFeedforward = 0.0;

    // The Pinpoint's heading from the last loop, so we can spot a jump (see
    // MAX_PINPOINT_HEADING_JUMP_DEGREES). haveLastPinpointHeading is false until we have one.
    private double lastPinpointHeadingDegrees = 0.0;
    private boolean haveLastPinpointHeading = false;

    // ---- Field-centric drive ----
    // Does the operator WANT field-centric? (The MODE button switches it.) It starts ON.
    private boolean fieldCentricWanted = true;

    // true = the safety rule switched field-centric off. It stays off until the operator's MODE
    // button or the driver's D-pad Left. The reason is shown on the screen.
    private boolean fieldCentricBlocked = false;
    private String fieldCentricBlockedReason = "";

    // Added to the Pinpoint's heading to get the heading the drive uses. It comes from the alliance
    // (see updateDriveFrameFromAlliance) until the driver re-aims with D-pad Left.
    private double fieldHeadingOffsetDegrees = 0.0;

    // The alliance the offset was worked out for. It is fixed at START, so it can be shown on the
    // screen even if the operator switches the alliance later (that only changes the camera's hive).
    private VisionSubsystem.Alliance driveFrameAlliance = VisionSubsystem.Alliance.BLUE;

    // true once the driver pressed D-pad Left: the offset is then the driver's, not the alliance's.
    private boolean driverReAimed = false;

    private boolean wasModeButtonPressed = false;
    private boolean wasReAimPressed = false;

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
        telemetry.addLine("--- GAMEPAD 1 (DRIVER) ---");
        telemetry.addLine("Left Stick = drive and strafe   Right Stick = turn");
        telemetry.addLine("Left Trigger = SLOW MODE");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
        telemetry.addLine("Right Trigger (hold) = SHOOT (the turret is always tracking the hive)");
        telemetry.addLine("D-pad Left = RE-AIM (the way the robot faces now is away from me)");
        telemetry.addLine("--- GAMEPAD 2 (OPERATOR) ---");
        telemetry.addLine("Start = alliance BLUE / RED   Back = flywheel speed DIAL / TABLE");
        telemetry.addLine("MODE button (guide or Left Stick Button) = drive FIELD-CENTRIC / ROBOT-CENTRIC");
        telemetry.addLine("D-pad Up / Down = flywheel RPM up / down");
        telemetry.addLine("D-pad Right / Left = ratio up / down");
        telemetry.addLine("X = gate CLOSED   B = gate OPENED   Y = gate LIFTED");
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE");
        telemetry.addLine("A (hold) = float the turret by hand (it tracks again when you let go)");
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
     * The words for the speed source switch, for the screen.
     */
    private String getModeText() {
        if (tableMode) {
            return "TABLE";
        }
        return "DIAL";
    }

    /*
     * Gamepad2 Start switches the alliance (BLUE <-> RED), and gamepad2 Back switches where
     * the flywheel speed comes from (DIAL <-> TABLE). One switch per press. This works
     * before and after START, so the operator can set both during INIT.
     */
    private void checkModeButtons() {
        boolean startPressed = gamepad2.start;
        boolean backPressed = gamepad2.back;

        if (isNewPress(backPressed, wasBackPressed)) {
            tableMode = !tableMode;
        }
        if (isNewPress(startPressed, wasStartPressed)) {
            if (visionSubsystem.getAlliance() == VisionSubsystem.Alliance.BLUE) {
                visionSubsystem.setAlliance(VisionSubsystem.Alliance.RED);
            } else {
                visionSubsystem.setAlliance(VisionSubsystem.Alliance.BLUE);
            }
            // The remembered hive direction belongs to the OTHER hive now. (setAlliance also
            // lets go of the vision's lock on a cell.)
            hiveMemory.clear();
        }

        wasStartPressed = startPressed;
        wasBackPressed = backPressed;
    }

    /*
     * The Pinpoint's heading helps when the camera cannot see the hive. Returns the words to
     * show on the screen. Call it only while auto-align is wanted and the camera has NO target.
     *
     * While the camera could see the hive, hiveMemory saved the hive's direction on the field.
     * Now we turn that into a turret angle for the way the robot is facing right now:
     *     turret angle = direction to the hive on the field - robot heading
     * so the turret keeps pointing at the hive while the robot spins.
     *
     * We only trust the memory while it is fresh: it goes STALE when the robot has driven too far
     * or too much time has passed (see HiveBearingMemory). A stale memory, a missing memory, or a
     * Pinpoint that is not READY all mean the same as before Alpha 9: hold still. It never searches.
     */
    private String aimWithoutCamera() {
        if (!pinpointSubsystem.isReady() || !hiveMemory.hasMemory()) {
            // Nothing to go on. If the turret is still running a manual stick power (the stick
            // was just let go) or is floating, hold where it is. If it already has a target
            // angle, it keeps working toward that angle.
            if (turretSubsystem.getState() != TurretSubsystem.TurretState.POSITION) {
                turretSubsystem.holdCurrentAngle();
            }
            return "AUTO-ALIGN (NO TARGET - holding)";
        }

        double robotX = pinpointSubsystem.getXInches();
        double robotY = pinpointSubsystem.getYInches();
        if (hiveMemory.isStale(robotX, robotY, timeSinceTarget.seconds())) {
            // Too old to trust, so stop where we are. (Setting the target to the angle we have
            // right now cancels any target the turret was still moving toward.)
            turretSubsystem.holdCurrentAngle();
            return "AUTO-ALIGN (NO TARGET - memory STALE, holding)";
        }

        double wantedAngle = hiveMemory.getTurretAngleDegrees(pinpointSubsystem.getHeadingDegrees());

        // The turret cannot go past its soft limits. If the hive is somewhere it cannot reach (for
        // example behind the robot), HOLD where we are. We do NOT push the target to the nearest
        // limit, because when the hive is behind the robot a tiny turn flips the answer from one
        // side to the other, and the turret would swing all the way across between the limits.
        if (wantedAngle < turretSubsystem.getMinAngleDegrees()
                || wantedAngle > turretSubsystem.getMaxAngleDegrees()) {
            turretSubsystem.holdCurrentAngle();
            return "AUTO-ALIGN (NO TARGET - hive is OUT OF the turret's range, holding)";
        }

        // Follow the robot's spin, the same as when the camera sees the hive, so the turret
        // does not trail behind. The target below moves with the heading every loop.
        turretFeedforward = -driveSubsystem.getTurnPower() * TURRET_FEEDFORWARD_GAIN;
        turretSubsystem.setFeedforwardPower(turretFeedforward);
        turretSubsystem.setTargetAngle(wantedAngle);
        return "AUTO-ALIGN (NO TARGET - following Pinpoint heading)";
    }

    /*
     * Throws away the remembered hive direction when the Pinpoint cannot be trusted, and switches
     * FIELD-CENTRIC drive off for the same reason (a bad heading would make the robot drive the
     * wrong way). Call it once per loop, right after pinpointSubsystem.update(). It only ever
     * switches field-centric OFF. Turning it back on is up to the operator's MODE button or the
     * driver's D-pad Left.
     *   - Pinpoint not READY: it is missing, still calibrating, or faulted.
     *   - Heading jumped: it changed by more than a robot can turn in one loop, so the Pinpoint
     *     restarted or glitched (a power dip can do this). Its heading is back at some new zero,
     *     so the remembered direction, which was measured against the OLD zero, is now wrong.
     */
    private void checkPinpointHealth() {
        if (!pinpointSubsystem.isReady()) {
            hiveMemory.clear();
            haveLastPinpointHeading = false;
            fieldCentricBlocked = true;
            fieldCentricBlockedReason = "the Pinpoint is not READY";
            return;
        }

        double heading = pinpointSubsystem.getHeadingDegrees();
        if (haveLastPinpointHeading) {
            double jump = Math.abs(HiveBearingMemory.wrapDegrees(heading - lastPinpointHeadingDegrees));
            if (jump > MAX_PINPOINT_HEADING_JUMP_DEGREES) {
                hiveMemory.clear();
                fieldCentricBlocked = true;
                fieldCentricBlockedReason = String.format("the Pinpoint heading jumped %.0f degrees", jump);
            }
        }
        lastPinpointHeadingDegrees = heading;
        haveLastPinpointHeading = true;
    }

    /*
     * The offset for the field-centric drive, worked out from the alliance. The heading the drive
     * uses is the Pinpoint's heading plus this: (which way the robot faces on the field at INIT) -
     * (which way the driver faces). Blue: -90 - 180 = -270, which wraps to +90. Red: +90 - 0 = +90.
     */
    private double getAllianceHeadingOffsetDegrees(VisionSubsystem.Alliance alliance) {
        double robotFieldHeadingAtInit;
        double driverForwardFieldHeading;
        if (alliance == VisionSubsystem.Alliance.RED) {
            robotFieldHeadingAtInit = ROBOT_FIELD_HEADING_AT_INIT_RED_DEGREES;
            driverForwardFieldHeading = DRIVER_FORWARD_FIELD_HEADING_RED_DEGREES;
        } else {
            robotFieldHeadingAtInit = ROBOT_FIELD_HEADING_AT_INIT_BLUE_DEGREES;
            driverForwardFieldHeading = DRIVER_FORWARD_FIELD_HEADING_BLUE_DEGREES;
        }
        return HiveBearingMemory.wrapDegrees(robotFieldHeadingAtInit - driverForwardFieldHeading);
    }

    /*
     * Sets the drive's offset from the CURRENT alliance, unless the driver has re-aimed (then the
     * driver's offset stays). Call it every loop in INIT, and once at START. It is NOT called after
     * START, so changing the alliance mid-match does not change which way the stick drives.
     */
    private void updateDriveFrameFromAlliance() {
        if (driverReAimed) {
            return;
        }
        driveFrameAlliance = visionSubsystem.getAlliance();
        fieldHeadingOffsetDegrees = getAllianceHeadingOffsetDegrees(driveFrameAlliance);
    }

    /*
     * True when the operator is pressing a MODE button candidate (see MODE_BUTTON_GUIDE_ENABLED and
     * MODE_BUTTON_LEFT_STICK_ENABLED).
     */
    private boolean isModeButtonPressed() {
        boolean pressed = false;
        if (MODE_BUTTON_GUIDE_ENABLED && gamepad2.guide) {
            pressed = true;
        }
        if (MODE_BUTTON_LEFT_STICK_ENABLED && gamepad2.left_stick_button) {
            pressed = true;
        }
        return pressed;
    }

    /*
     * The operator's MODE button and the driver's D-pad Left. Call it once per loop.
     *   MODE: if the safety rule switched field-centric off, turn it back ON. Otherwise switch
     *         between FIELD-CENTRIC and ROBOT-CENTRIC.
     *   D-pad Left (RE-AIM): "the way the robot faces right now is away from me". It only works
     *         while the Pinpoint is READY (it needs the heading). The Pinpoint is NOT reset. It also
     *         turns field-centric back on if the safety rule had switched it off.
     */
    private void checkDriveModeButtons() {
        boolean modePressed = isModeButtonPressed();
        boolean reAimPressed = gamepad1.dpad_left;

        if (isNewPress(modePressed, wasModeButtonPressed)) {
            if (fieldCentricBlocked) {
                // Resume. (If the Pinpoint is still not READY, the safety rule switches it off
                // again on the next loop, and the screen says so.)
                fieldCentricBlocked = false;
                fieldCentricWanted = true;
            } else {
                fieldCentricWanted = !fieldCentricWanted;
            }
        }

        if (isNewPress(reAimPressed, wasReAimPressed) && pinpointSubsystem.isReady()) {
            // Make the heading the drive uses ZERO right now: the robot faces away from the driver.
            fieldHeadingOffsetDegrees = -pinpointSubsystem.getHeadingDegrees();
            driverReAimed = true;
            fieldCentricBlocked = false;
        }

        wasModeButtonPressed = modePressed;
        wasReAimPressed = reAimPressed;
    }

    /*
     * Is field-centric being used right now? The operator wants it, the safety rule has not
     * switched it off, and the Pinpoint is READY.
     */
    private boolean isFieldCentricActive() {
        return fieldCentricWanted && !fieldCentricBlocked && pinpointSubsystem.isReady();
    }

    /*
     * The words for the "DRIVE MODE" line on the screen.
     */
    private String getDriveModeText() {
        if (!fieldCentricWanted) {
            return "ROBOT-CENTRIC (the operator's MODE button switches to FIELD-CENTRIC)";
        }
        if (fieldCentricBlocked) {
            return "ROBOT-CENTRIC: field-centric was switched off because " + fieldCentricBlockedReason
                    + ". The operator's MODE button or the driver's D-pad Left turns it on again.";
        }
        if (!pinpointSubsystem.isReady()) {
            return "ROBOT-CENTRIC for now (the Pinpoint is not READY yet)";
        }
        if (driverReAimed) {
            return "FIELD-CENTRIC (re-aimed: forward = the way the robot faced when D-pad Left was pressed)";
        }
        if (driveFrameAlliance == VisionSubsystem.Alliance.RED) {
            return "FIELD-CENTRIC (RED driver station: forward = toward the RIGHT wall)";
        }
        return "FIELD-CENTRIC (BLUE driver station: forward = toward the LEFT wall)";
    }

    /*
     * The words for the "Hive memory" line on the screen: what the Pinpoint remembers about the
     * hive, and whether the turret can use it right now.
     */
    private String getHiveMemoryText() {
        if (!pinpointSubsystem.isReady()) {
            return "unavailable (the Pinpoint is not READY)";
        }
        if (!hiveMemory.hasMemory()) {
            return "none yet (the camera has not seen the hive)";
        }
        double robotX = pinpointSubsystem.getXInches();
        double robotY = pinpointSubsystem.getYInches();
        double secondsSinceSeen = timeSinceTarget.seconds();
        String useText;
        if (visionSubsystem.hasTarget()) {
            useText = "LIVE (the camera sees the hive)";
        } else if (hiveMemory.isStale(robotX, robotY, secondsSinceSeen)) {
            useText = "STALE";
        } else {
            useText = "FOLLOWING";
        }
        return String.format("%s   hive %.0f deg on field, seen %.1f of %.0f s ago, driven %.1f of %.0f in",
                useText, hiveMemory.getHiveFieldHeadingDegrees(),
                secondsSinceSeen, hiveMemory.getMaxHoldSeconds(),
                hiveMemory.getDrivenInchesSinceSeen(robotX, robotY),
                hiveMemory.getMaxDriveSinceSeenInches());
    }

    /*
     * Turns the turret toward the hive. Returns the words to show on the screen.
     * Call it every loop while ALWAYS-TRACK is ON or the driver's Right Trigger is held.
     */
    private String autoAlignTurret() {
        if (!visionSubsystem.hasTarget()) {
            // The camera does not see the hive. The Pinpoint may still know which way it is.
            return aimWithoutCamera();
        }

        double txDegrees = visionSubsystem.getTxDegrees();

        // Follow the robot's spin. The drive's turn power is positive for COUNTERCLOCKWISE (left).
        // When the robot spins left the turret has to turn RIGHT (negative) to keep pointing at
        // the hive, so the push is the turn power with its sign flipped. getTurnPower() already
        // includes slow mode. It is 0.0 when the driver is not turning.
        turretFeedforward = -driveSubsystem.getTurnPower() * TURRET_FEEDFORWARD_GAIN;
        turretSubsystem.setFeedforwardPower(turretFeedforward);
        boolean robotIsTurning = turretFeedforward != 0.0;

        // Pick a new target if the turret is not in POSITION mode yet (the align just started),
        // or if the camera says we are far enough off. While the robot is turning we pick a new
        // target EVERY loop: the push is already moving the turret, so the target has to move
        // with it, or position control would pull the turret back toward an old target.
        boolean notAimingYet = turretSubsystem.getState() != TurretSubsystem.TurretState.POSITION;
        boolean farOff = Math.abs(txDegrees) > ALIGN_REAIM_THRESHOLD_DEGREES;
        if (notAimingYet || farOff || robotIsTurning) {
            // tx is positive to the RIGHT, but turret angles are positive to the LEFT, so subtract.
            double newTargetAngle = turretSubsystem.getAngleDegrees() - txDegrees;
            turretSubsystem.setTargetAngle(newTargetAngle);
        }
        if (robotIsTurning) {
            return "AUTO-ALIGN (following the robot's spin)";
        }
        return "AUTO-ALIGN";
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Each subsystem gets its own motors from the hardware map.
        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        // The gate servo moves to CLOSED here, so it starts from a known place.
        shooterServoSubsystem = new ShooterServoSubsystem(hardwareMap);
        // The turret does not move at INIT. It only starts reading its encoder.
        turretSubsystem = new TurretSubsystem(hardwareMap);
        // The Limelight starts polling here. It needs about 30 seconds to boot after power-up.
        visionSubsystem = new VisionSubsystem(hardwareMap);
        // The Pinpoint zeros its position and calibrates its gyro here, so the robot must be
        // STILL during INIT. If the config has no "pinpoint", this does not stop the OpMode.
        pinpointSubsystem = new PinpointSubsystem(hardwareMap);
        hiveMemory = new HiveBearingMemory();
        shotTable = new ShotTable();

        telemetry.addData("Status", "Initialized");
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Read the camera and the operator's mode buttons now, so the alliance and the speed
        // source can be set (and checked) before START.
        visionSubsystem.update();
        pinpointSubsystem.update();
        checkModeButtons();

        // The drive: the alliance (just checked above) decides which way "away from the driver" is.
        // The MODE button and D-pad Left already work here, so you can try them before START.
        updateDriveFrameFromAlliance();
        checkDriveModeButtons();

        // The Driver Station clears its screen every loop, so we add the
        // button map again here. That keeps it on the screen until START.
        telemetry.addData("Status", "Ready to start!");
        telemetry.addData("Alliance (gamepad2 Start)", visionSubsystem.getAlliance());
        telemetry.addData("DRIVE MODE (operator MODE button)", getDriveModeText());
        // Which button is MODE? Press them. The one that says true is the one that works.
        telemetry.addData("MODE button test (gamepad2)", "guide = %b    left stick button = %b",
                gamepad2.guide, gamepad2.left_stick_button);
        telemetry.addData("Flywheel speed source (gamepad2 Back)", getModeText());
        visionSubsystem.addTelemetry(telemetry);
        // Check that the Pinpoint says READY before START. (The robot must be still until then.)
        pinpointSubsystem.addTelemetry(telemetry);
        addRawGamepadTelemetry();
        addControlsTelemetry();

        // Show the gate here too, so you can check before START that it went to CLOSED.
        // (The position shown is the one we COMMANDED. A servo cannot report where it really is.)
        shooterServoSubsystem.addTelemetry(telemetry);

        // The turret is not driven during INIT. We only read it, so you can turn it by hand
        // and watch the angle.
        turretSubsystem.update();
        turretSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        loopTimer.reset();

        // Fix the drive's offset from the alliance as it is NOW. From here on, switching the alliance
        // only changes the camera's hive, not which way the stick drives. The safety rule only
        // counts from START (the Pinpoint may still have been calibrating in INIT).
        updateDriveFrameFromAlliance();
        fieldCentricBlocked = false;
        fieldCentricBlockedReason = "";
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 0. Time the loop. This is how long it has been since the last loop() started. The very
        //    first loop only measures the wait after START, so we leave it out of the average
        //    and the worst. (Run the robot a few seconds before you read the average.)
        double thisLoopMs = loopTimer.milliseconds();
        loopTimer.reset();
        loopCount = loopCount + 1;
        if (loopCount > 1) {
            totalLoopMs = totalLoopMs + thisLoopMs;
            if (thisLoopMs > worstLoopMs) {
                worstLoopMs = thisLoopMs;
            }
        }

        // 1. Read sensors. (The drive has none. The intake reads its own encoder in update().
        //    The shooter reads its wheel encoders in its own update(), in step 3 below.)
        intakeSubsystem.update();
        visionSubsystem.update();
        pinpointSubsystem.update();
        checkPinpointHealth();

        // 2. Read the gamepads and decide what to do.
        checkModeButtons();
        checkDriveModeButtons();

        // ---- Gamepad 1 (driver) ----
        // On the controller, pushing a stick FORWARD gives a negative number, and pushing it RIGHT
        // gives a positive number. Our subsystem wants forward +, LEFT +, and COUNTERCLOCKWISE +,
        // so we flip the sign of all three. (Turning right is clockwise, which is negative.)
        double forward = applyDeadband(-gamepad1.left_stick_y);
        double strafe = applyDeadband(-gamepad1.left_stick_x);
        double turn = applyDeadband(-gamepad1.right_stick_x);

        boolean slowMode = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean triggerShotPressed = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;

        // ---- Gamepad 2 (operator) ----
        // The operator's face buttons move the shooter gate. (A is the turret float button.)
        boolean gateClosedPressed = gamepad2.x;
        boolean gateOpenedPressed = gamepad2.b;
        boolean gateLiftedPressed = gamepad2.y;
        boolean turretFloatPressed = gamepad2.a;
        boolean dpadUpPressed = gamepad2.dpad_up;
        boolean dpadDownPressed = gamepad2.dpad_down;
        boolean dpadLeftPressed = gamepad2.dpad_left;
        boolean dpadRightPressed = gamepad2.dpad_right;

        // Turn the dials. Only a NEW press counts, so holding a button moves it one step.
        if (isNewPress(dpadUpPressed, wasDpadUpPressed)) {
            flywheelRpmDial = flywheelRpmDial + FLYWHEEL_RPM_STEP;
        }
        if (isNewPress(dpadDownPressed, wasDpadDownPressed)) {
            flywheelRpmDial = flywheelRpmDial - FLYWHEEL_RPM_STEP;
        }
        if (isNewPress(dpadRightPressed, wasDpadRightPressed)) {
            ratioHundredths = ratioHundredths + RATIO_HUNDREDTHS_STEP;
        }
        if (isNewPress(dpadLeftPressed, wasDpadLeftPressed)) {
            ratioHundredths = ratioHundredths - RATIO_HUNDREDTHS_STEP;
        }

        // Keep the dials between their lowest and highest settings.
        if (flywheelRpmDial > FLYWHEEL_RPM_MAX) {
            flywheelRpmDial = FLYWHEEL_RPM_MAX;
        }
        if (flywheelRpmDial < FLYWHEEL_RPM_MIN) {
            flywheelRpmDial = FLYWHEEL_RPM_MIN;
        }
        if (ratioHundredths > RATIO_HUNDREDTHS_MAX) {
            ratioHundredths = RATIO_HUNDREDTHS_MAX;
        }
        if (ratioHundredths < RATIO_HUNDREDTHS_MIN) {
            ratioHundredths = RATIO_HUNDREDTHS_MIN;
        }

        // The hood wheel target is the flywheel target times the ratio. The shooter will not go
        // past its maximum RPM, so if the answer is more than that we show the maximum and warn
        // that it was CAPPED. (The shooter cuts it down by itself, we only warn about it.)
        double ratio = ratioHundredths / HUNDREDTHS_PER_ONE;

        // Where the flywheel speed comes from. While the camera sees the hive we work out the
        // table's RPM every loop and remember it. DIAL mode always uses the dial. TABLE mode uses
        // the table's RPM, and if the camera loses the hive it keeps the LAST table RPM for
        // TABLE_RPM_HOLD_SECONDS before it goes back to the dial.
        boolean hasTarget = visionSubsystem.hasTarget();
        if (hasTarget) {
            lastTableRpm = shotTable.getFlywheelRpm(visionSubsystem.getTyDegrees());
            haveTableRpm = true;
            timeSinceTarget.reset();

            // Remember which way the hive is on the FIELD, using the Pinpoint's heading, so the
            // turret can keep pointing that way if the camera loses the hive. We do this every loop
            // the hive is in view, whether or not auto-align is on, so the memory is fresh the
            // moment it is needed. (The turret angle is from the last loop, because the turret
            // updates later in this loop. It moves only a fraction of a degree in one loop.)
            if (pinpointSubsystem.isReady()) {
                hiveMemory.remember(
                        pinpointSubsystem.getHeadingDegrees(),
                        turretSubsystem.getAngleDegrees(),
                        visionSubsystem.getTxDegrees(),
                        pinpointSubsystem.getXInches(),
                        pinpointSubsystem.getYInches());
            }
        }

        double flywheelTargetRpm = flywheelRpmDial;
        String rpmSource = "DIAL";
        if (tableMode) {
            if (hasTarget) {
                flywheelTargetRpm = lastTableRpm;
                rpmSource = "TABLE";
            } else if (haveTableRpm && timeSinceTarget.seconds() <= TABLE_RPM_HOLD_SECONDS) {
                flywheelTargetRpm = lastTableRpm;
                rpmSource = "TABLE (camera lost the hive, holding the last RPM)";
            } else {
                rpmSource = "DIAL (TABLE mode, but the camera has not seen the hive)";
            }
        }
        double wantedHoodWheelRpm = flywheelTargetRpm * ratio;
        double maxFlywheelRpm = shooterSubsystem.getMaxFlywheelTargetRpm();
        double maxHoodWheelRpm = shooterSubsystem.getMaxHoodWheelTargetRpm();
        boolean flywheelCapped = flywheelTargetRpm > maxFlywheelRpm + CAPPED_TOLERANCE_RPM;
        boolean hoodWheelCapped = wantedHoodWheelRpm > maxHoodWheelRpm + CAPPED_TOLERANCE_RPM;
        double hoodWheelTargetRpm = Math.min(wantedHoodWheelRpm, maxHoodWheelRpm);

        // 3. Tell the subsystems what to do. (The drive has no update() to call.)

        // Drive.
        // How slow "slow mode" is lives in DriveSubsystem (SLOW_MODE_POWER_SCALE).
        driveSubsystem.setSlowMode(slowMode);
        String driveControl;
        if (slowMode) {
            driveControl = "SLOW MODE";
        } else {
            driveControl = "Normal";
        }

        // FIELD-CENTRIC: the stick means "away from the driver", so the drive needs to know how far
        // the robot is turned from that: the Pinpoint's heading plus the offset. ROBOT-CENTRIC: the
        // stick means "the way the robot faces". (The turn stick is the same in both.)
        boolean fieldCentricActive = isFieldCentricActive();
        double drivingHeadingDegrees = 0.0;
        if (fieldCentricActive) {
            drivingHeadingDegrees = pinpointSubsystem.getHeadingDegrees() + fieldHeadingOffsetDegrees;
            driveSubsystem.driveFieldCentric(forward, strafe, turn, drivingHeadingDegrees);
            driveControl = driveControl + ", FIELD-CENTRIC";
        } else {
            driveSubsystem.drive(forward, strafe, turn);
            driveControl = driveControl + ", ROBOT-CENTRIC";
        }

        // Intake (the bumpers on BOTH gamepads do the same thing: Right = INTAKE, Left = REVERSE).
        // Check REVERSE first, so clearing a jam always works, even if the other gamepad is
        // holding INTAKE.
        String intakeControl;
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            intakeSubsystem.reverse();
            intakeControl = "Left Bumper (REVERSE)";
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            intakeSubsystem.intake();
            intakeControl = "Right Bumper (INTAKE)";
        } else {
            // No bumper held, so stop the intake.
            intakeControl = "None";
            intakeSubsystem.stop();
        }

        // Shooter wheels. The driver holds the Right Trigger to shoot. (There is no wheel reverse
        // in this OpMode any more.)
        String shooterControl;
        if (triggerShotPressed) {
            // The driver holds the Right Trigger to shoot, at the RPMs the operator dialed in.
            // The shooter's speed control holds each wheel at its target, even when a ball
            // slows it down or the battery drains.
            shooterSubsystem.setTargetRpms(flywheelTargetRpm, hoodWheelTargetRpm);
            shooterControl = "Right Trigger (SHOOT)";
        } else {
            // Trigger not held, so both wheels get 0 power and coast down.
            shooterSubsystem.stop();
            shooterControl = "OFF";
        }
        shooterSubsystem.update();

        // Shooter gate. A button counts as ONE press only on the loop where it goes from
        // "not pressed" to "pressed", so holding it does not keep re-sending the move.
        // The gate is the operator's (gamepad2): X = CLOSED, B = OPENED, Y = LIFTED.
        // If more than one is pressed on the same loop, CLOSED wins, then OPENED, then LIFTED.
        boolean closedNewPress = isNewPress(gateClosedPressed, wasGateClosedPressed);
        boolean openedNewPress = isNewPress(gateOpenedPressed, wasGateOpenedPressed);
        boolean liftedNewPress = isNewPress(gateLiftedPressed, wasGateLiftedPressed);
        if (closedNewPress) {
            shooterServoSubsystem.goToClosed();
        } else if (openedNewPress) {
            shooterServoSubsystem.goToOpened();
        } else if (liftedNewPress) {
            shooterServoSubsystem.goToLifted();
        }

        // Turret. It tracks the hive ALL THE TIME (ALWAYS-TRACK, no switch). The only operator
        // control is A: HOLD it to float the turret so it can be turned by hand. Let go and the
        // turret goes back to tracking. (stop() leaves the turret in IDLE, so auto-align picks a
        // fresh target the moment A is released.)

        // Cleared every loop. Only auto-align sets it, so nothing else ever gets a push.
        turretFeedforward = 0.0;

        String turretControl;
        if (turretFloatPressed) {
            turretSubsystem.stop();
            turretControl = "A (FLOATING - let go to track again)";
        } else {
            turretControl = autoAlignTurret();
        }
        turretSubsystem.update();

        // Remember this loop's buttons for the next loop.
        wasDpadUpPressed = dpadUpPressed;
        wasDpadDownPressed = dpadDownPressed;
        wasGateClosedPressed = gateClosedPressed;
        wasGateOpenedPressed = gateOpenedPressed;
        wasGateLiftedPressed = gateLiftedPressed;
        wasDpadLeftPressed = dpadLeftPressed;
        wasDpadRightPressed = dpadRightPressed;

        // 4. Show what is happening on the Driver Station, one section at a time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        if (loopCount > 1) {
            double averageLoopMs = totalLoopMs / (loopCount - 1);
            telemetry.addData("Loop time (ms)", "%.1f now   %.1f average   %.1f worst   (%.0f loops/sec)",
                    thisLoopMs, averageLoopMs, worstLoopMs, 1000.0 / averageLoopMs);
        }
        // Always on the screen: aiming at the wrong alliance's hive is not an obvious failure.
        telemetry.addData("Alliance (gamepad2 Start)", visionSubsystem.getAlliance());
        telemetry.addData("DRIVE MODE (operator MODE button)", getDriveModeText());

        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive Control", driveControl);
        if (fieldCentricActive) {
            telemetry.addData("Heading the drive uses (deg)", "%.1f   (Pinpoint %.1f + offset %.1f)",
                    drivingHeadingDegrees, pinpointSubsystem.getHeadingDegrees(), fieldHeadingOffsetDegrees);
        }
        telemetry.addData("Drive inputs", "forward %.2f   strafe %.2f   turn %.2f",
                forward, strafe, turn);

        // The Pinpoint prints its own "--- ODOMETRY (Pinpoint) ---" heading, with its status, the
        // robot's position (X, Y, heading) and its speeds. Nothing uses the position yet.
        pinpointSubsystem.addTelemetry(telemetry);

        // The dials: what the operator has asked for. (They only take effect while the driver
        // holds the Right Trigger.)
        telemetry.addLine("--- SHOOTER DIALS (RPM) ---");
        telemetry.addData("Flywheel speed source (gamepad2 Back)", getModeText());
        telemetry.addData("Using", rpmSource);
        if (flywheelCapped) {
            telemetry.addData("Flywheel target RPM", "%.0f  CAPPED at %.0f (you asked for %.0f)",
                    maxFlywheelRpm, maxFlywheelRpm, flywheelTargetRpm);
        } else {
            telemetry.addData("Flywheel target RPM", "%.0f   (the dial is at %d)",
                    flywheelTargetRpm, flywheelRpmDial);
        }
        telemetry.addData("Ratio (hood / flywheel)", "%.2f", ratio);
        if (hoodWheelCapped) {
            telemetry.addData("Hood wheel target RPM", "%.0f  CAPPED at %.0f (you asked for %.0f)",
                    hoodWheelTargetRpm, maxHoodWheelRpm, wantedHoodWheelRpm);
        } else {
            telemetry.addData("Hood wheel target RPM", "%.0f", hoodWheelTargetRpm);
        }
        telemetry.addData("Shooter switch", shooterControl);

        // The shooter prints its own "--- SHOOTER ---" heading, with its state, "At Speed?",
        // and each wheel's real RPM next to its target. This is the data from the encoders.
        shooterSubsystem.addTelemetry(telemetry);

        // Is it OK to shoot? This comes from the real wheel speeds, not from a timer.
        ShooterSubsystem.ShooterState shooterState = shooterSubsystem.getState();
        String readyText;
        if (shooterState == ShooterSubsystem.ShooterState.READY) {
            readyText = "YES - OK to shoot";
        } else if (shooterState == ShooterSubsystem.ShooterState.SPINNING_UP) {
            readyText = "WAIT - wheels are not at their target RPM yet";
        } else {
            readyText = "shooter is OFF (hold the Right Trigger to shoot)";
        }
        telemetry.addData("OK to shoot?", readyText);

        // The hood wheel RPM / flywheel RPM we really have right now. It should match the ratio
        // dial once the wheels are at speed. If it does not, the hood wheel may be CAPPED.
        if (shooterSubsystem.getFlywheelRpm() > MIN_RPM_FOR_MEASURED_RATIO) {
            telemetry.addData("Ratio measured now", "%.2f",
                    shooterSubsystem.getHoodWheelRpm() / shooterSubsystem.getFlywheelRpm());
        }

        // The gate prints its own "--- SHOOTER GATE ---" heading.
        shooterServoSubsystem.addTelemetry(telemetry);

        // The intake prints its own "--- INTAKE ---" heading, so we add our line under it.
        intakeSubsystem.addTelemetry(telemetry);
        telemetry.addData("Intake Control", intakeControl);

        // The turret prints its own "--- TURRET ---" heading, so we add our lines under it.
        turretSubsystem.addTelemetry(telemetry);
        telemetry.addData("Turret Control", turretControl);
        telemetry.addData("ALWAYS-TRACK", "ON - the turret follows the hive (hold gamepad2 A to float it)");
        telemetry.addData("Turret follows spin", "%+.2f power   (gain %.2f)",
                turretFeedforward, TURRET_FEEDFORWARD_GAIN);
        telemetry.addData("Hive memory (Pinpoint)", getHiveMemoryText());

        // The vision prints its own "--- VISION ---" heading, so we add our lines under it.
        visionSubsystem.addTelemetry(telemetry);
        if (hasTarget) {
            telemetry.addData("Table says", "%.0f flywheel RPM", lastTableRpm);
            if (!shotTable.isInRange(visionSubsystem.getTyDegrees())) {
                telemetry.addLine("WARNING: ty is OUTSIDE the table. It is using the nearest row.");
            }
        }

        // Is the turret's half of the firing gate open? The camera must say we are centered on
        // the hive, AND the turret must say it has reached its target and settled.
        boolean aimed = hasTarget
                && Math.abs(visionSubsystem.getTxDegrees()) <= AIMED_TOLERANCE_DEGREES;
        String aimText;
        if (!hasTarget) {
            aimText = "NO TARGET";
        } else if (aimed && turretSubsystem.isOnTarget()) {
            aimText = "YES - turret is aimed";
        } else if (aimed) {
            aimText = "almost - camera is centered, turret is still settling";
        } else {
            aimText = String.format("NO - off target by %.1f deg (tx)",
                    visionSubsystem.getTxDegrees());
        }
        telemetry.addData("Turret aimed?", aimText);

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
        // Always leave the motors stopped.
        // The gate servo just stays where it is. A servo holds its position by itself.
        driveSubsystem.stop();
        intakeSubsystem.stop();
        shooterSubsystem.stop();
        turretSubsystem.stop();
        // Stop polling the camera.
        visionSubsystem.stop();
        // The Pinpoint only reads, so there is nothing to stop.
    }
}
