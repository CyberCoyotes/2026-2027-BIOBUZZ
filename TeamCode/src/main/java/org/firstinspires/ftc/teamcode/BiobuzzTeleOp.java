package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FiringSequence;
import org.firstinspires.ftc.teamcode.subsystems.HiveBearingMemory;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * TeleOp Alpha 13
 * An iterative OpMode with TWO controllers:
 *   gamepad1 = the DRIVER: drives the robot, runs the INTAKE, and holds the trigger to FIRE a volley
 *              (the FIRING SEQUENCE, see NEW IN ALPHA 13).
 *              (The turret AUTO-TRACKS the hive ALL THE TIME, see ALWAYS-TRACK below.)
 *   gamepad2 = the OPERATOR: sets the shooter speeds (RPM), moves the GATE, and can run the
 *              INTAKE too. The operator does NOT aim the turret by hand any more.
 * It is DriveAlpha, IntakeAlpha, ShooterRPMAlpha, TurretAlpha, VisionAlpha, ShotTableAlpha and
 * PinpointAlpha put together, so each part has already been tested on its own. This OpMode owns
 * NO hardware. It only reads the gamepads and tells the subsystems what to do.
 *
 * NEW IN ALPHA 13: THE FIRING SEQUENCE (issue #63). The screen name became "TeleOp - Alpha 13" (it was
 * Alpha 12). The three Autos' preselectTeleOp use the new name, so the Driver Station still loads
 * this OpMode after Auto.
 *   Before, the driver held the Right Trigger to spin the wheels and the operator opened and closed
 *   the gate by hand, so a ball could go through before the wheels were at speed. Now FiringSequence
 *   (a plain class, see subsystems/FiringSequence.java) runs a whole VOLLEY by itself:
 *     HOLD the OPERATOR's Right Trigger (gamepad2) -> the wheels spin (if they are not already). The
 *     volley WAITS
 *     until the flywheel is at speed AND the camera says the turret is aimed (tx within 2 degrees for
 *     0.15 s) AND (in TABLE mode) the camera sees the hive right now. (With no camera in DIAL mode the
 *     turret just has to be holding still.) Then the gate OPENS and the intake feeds the balls one at a time (it runs
 *     only while the flywheel is at speed), then the gate CLOSES. (There is no LIFT step any more:
 *     its timing was off and it wedged the last ball.)
 *     Let go of the trigger at any time to ABORT: the gate closes and the intake stops. A held
 *     trigger fires ONE volley. Let go and press again to fire another.
 *   WHO FIRES: the OPERATOR's Right Trigger (gamepad2). The driver's Right Trigger does NOT fire, unless
 *     SOLO MODE is ON. SOLO MODE is for a match with ONE person driving: the driver presses gamepad1
 *     LOGO (the center "guide" button) to switch it ON / OFF (it starts OFF, and it works in INIT too).
 *     While it is ON the driver's Right Trigger fires too, and the screen says SOLO MODE ON. In a
 *     solo match nobody can use the operator's buttons (the gate buttons, the dials, DIAL / TABLE),
 *     so set the speed source with gamepad 2 (LOGO) first if you can. (The driver CAN set the
 *     alliance with START and switch field / robot centric with BACK, see below.) The LOGO button is
 *     known to reach the Robot Controller on our Logitech controllers.
 *   PRE-REV: the driver's D-pad Up switches the wheels ON / OFF while you drive to the shooting
 *     spot, so the 4 seconds of spin-up are not paid at every volley. It starts OFF, and the screen
 *     says PRE-REV ON. It does not touch the gate or the intake.
 *   The gate buttons (operator X / B / Y) are an OVERRIDE: they only work BETWEEN volleys. During a
 *     volley the sequence is in charge of the gate.
 *   The intake is the DRIVER's only (gamepad1 bumpers; the operator's bumpers do nothing). REVERSE (Left
 *     Bumper) beats the volley, which beats INTAKE (Right Bumper). (If you clear a jam in the middle of a
 *     volley, the feed clock keeps running, so let go and press again.)
 *   GATE CLOSING AUTO-REVERSE: any time the gate walks from OPENED to CLOSED, or from LIFTED to CLOSED,
 *     the intake runs in REVERSE for exactly that long (about 0.75 s from OPENED, about 2 s from LIFTED),
 *     so a ball in the gate's way is pushed back out instead of being pulled in against the closing
 *     gate. It happens after every volley, on an abort, and on the manual X button. It beats the
 *     driver's INTAKE button while the gate is closing. The screen says "gate closing: REVERSE
 *     (automatic)". An instant close (STOP) does not do it: STOP stops everything.
 *   The READY check uses the FLYWHEEL ONLY (ShooterSubsystem.isFlywheelAtSpeed()), like
 *     ShootAndParkAuto, and the screen shows the hood wheel's error. The "OK to shoot?" line on the
 *     screen is still the old both-wheels check, for information.
 *   The flywheel target and the ratio are FROZEN during the feed and the end of a volley.
 *     In the other steps the target only changes when the wanted RPM moves by more than
 *     RPM_CHANGE_DEADBAND, so a wobbling table answer cannot keep the "at speed" check from ever
 *     holding.
 *   The timing (FEED_SECONDS) lives in FiringSequence, not here.
 *   NOT TESTED ON THE ROBOT YET: whether all 4 balls leave, and the timings. See issue #63.
 *
 * NEW IN ALPHA 12: CONTROL CHANGES (issue #74). The screen name became "TeleOp - Alpha 12" (it was
 * Alpha 11, and is Alpha 13 now).
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
 *   - THE DRIVE TOGGLE (FIELD-CENTRIC <-> ROBOT-CENTRIC) is BACK on BOTH gamepads. The drive mode is
 *     on the screen next to the alliance line, big.
 *     THE PHYSICAL "MODE" BUTTON cannot be used: the FTC SDK's Gamepad class has no field for it, and
 *     Logitech says MODE is a hardware switch that swaps the D-pad and the left stick (its LED
 *     lights when swapped). The INIT screen has a "BUTTON TEST" that shows every button each gamepad
 *     reports. WARNING: do not press MODE on the driver's controller while driving. If its LED is
 *     on, the left stick reads as the D-pad and the robot will not drive. Press MODE again.
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
 *   - THE SAFETY RULE: if the Pinpoint is not READY for MORE than PINPOINT_GLITCH_TOLERANCE_SECONDS
 *     (0.25 s) after START, or its heading jumps (the same check that protects the hive memory),
 *     field-centric switches ITSELF OFF and the robot drives ROBOT-CENTRIC. The screen says why. It
 *     does NOT switch itself back on, because a restarted Pinpoint has a new zero. The drive toggle
 *     (BACK on either gamepad) or the driver's D-pad Left turns it on again.
 *     GLITCH DEBOUNCE (added after the first robot test, where a weak battery made the Pinpoint blink
 *     and field-centric kept switching off): a gap SHORTER than 0.25 s is a glitch, not a restart. The
 *     drive keeps the LAST good heading (HOLD_LAST_HEADING_DURING_GLITCH), the hive memory is kept, and
 *     when the Pinpoint is READY again its heading must be within RESUME_MAX_HEADING_CHANGE_DEGREES
 *     (20) of the last good one, or we assume it restarted and switch field-centric off after all.
 *     The screen counts the glitches ("Pinpoint glitches"). Many of them means a weak battery.
 *     THE CAMERA: VisionSubsystem now checks every 0.5 s that the Limelight is on the AprilTag
 *     pipeline, and switches it back if it restarted into another one ("Camera pipeline fixes").
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
 *      (or AUDIENCE). There is NO new button. The alliance button (START) also clears
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
 *       CLOSED wins, then OPENED, then LIFTED. They only work BETWEEN volleys: during a volley the
 *       firing sequence is in charge of the gate, see NEW IN ALPHA 13.)
 *   A -> HOLD to STOP the turret (it floats, so you can turn it by hand). Let go and it goes
 *        back to tracking the hive.
 *   D-pad Up / Down  -> flywheel target RPM +/- FLYWHEEL_RPM_STEP
 *   D-pad Right / Left -> ratio (hood wheel / flywheel) +/- 0.05
 *   Right Trigger    -> HOLD to FIRE a volley (the FIRING SEQUENCE, see NEW IN ALPHA 13). The turret is
 *                       already tracking the hive (ALWAYS-TRACK). The wheels run at the flywheel
 *                       speed from the DIAL or the TABLE (see THE SHOT TABLE below), and the hood
 *                       wheel at that x the ratio dial. Let go to ABORT: the gate closes, the
 *                       intake stops, and (unless PRE-REV is ON) both wheels go to 0 power. A (hold)
 *                       floats the turret, so then a volley waits (the screen says it is floating).
 *   Start            -> switch alliance BLUE <-> RED (starts as BLUE). Check it on the screen!
 *                       (Gamepad1's START does the same.)
 *   Back             -> switch the DRIVE FIELD-CENTRIC <-> ROBOT-CENTRIC (starts FIELD-CENTRIC). If
 *                       field-centric was switched off by the safety rule, it turns it back ON.
 *                       (Gamepad1's BACK does the same.)
 *   LOGO             -> switch the flywheel speed source DIAL <-> TABLE (starts as DIAL)
 *   Both sticks, both bumpers, Left Trigger: not used. (The intake is the driver's only.)
 *   The turret has no other operator control. It tracks the hive all the time (see ALWAYS-TRACK
 *   below). It will not go past +/-120 degrees (see TurretSubsystem). 0 degrees is where
 *   the turret pointed when the Control Hub powered on, because the home switch is not
 *   installed yet, so power on with the turret pointing straight ahead.
 *
 * It starts FIELD-CENTRIC: "forward" means AWAY FROM THE DRIVER (see NEW IN ALPHA 11). ROBOT-CENTRIC
 * ("forward" means the way the robot is facing) is one press of the drive toggle away.
 *
 * GAMEPAD 1 - DRIVER
 *   Left Stick   -> drive forward / backward and slide (strafe) left / right
 *   Right Stick  -> turn (push right to turn right)
 *   HOLD the Left Trigger -> SLOW MODE for careful lining up
 *   D-pad Left   -> RE-AIM: "the way the robot faces right now is away from me" (field-centric).
 *      It does not touch the Pinpoint. It also turns field-centric back on after the safety rule.
 *   D-pad Up     -> PRE-REV on / off: the wheels spin without firing (see NEW IN ALPHA 13).
 *   START        -> switch alliance BLUE <-> RED, the same as gamepad2's START.
 *   BACK         -> switch the drive FIELD-CENTRIC <-> ROBOT-CENTRIC, the same as gamepad2's BACK.
 *   Right Bumper -> INTAKE (pull balls in)      Left Bumper -> REVERSE (push balls out)
 *      (Left Bumper beats Right Bumper, so clearing a jam always works. Only the DRIVER has the
 *       intake: the operator's bumpers do nothing. The intake runs at the power set in
 *       IntakeSubsystem, DEFAULT_INTAKE_POWER.)
 *   LOGO (the center "guide" button) -> SOLO MODE on / off, for a match with one person (see NEW IN
 *      ALPHA 13). It starts OFF.
 *   Right Trigger -> does NOT fire. The OPERATOR's Right Trigger fires. Only in SOLO MODE (LOGO) does
 *      the driver's Right Trigger fire too. At the start the shot is DIAL mode, flywheel 1300 RPM and
 *      hood wheel 2600 RPM (a ratio of 2.00).
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
 * THE SHOT TABLE (TABLE mode, switched with gamepad2 LOGO)
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
 *   - The ALLIANCE (START on either gamepad) must be right, or the turret tracks the OTHER team's hive.
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
@TeleOp(name = "TeleOp - Alpha 13", group = "Testing")
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
    // THE DRIVE TOGGLE (FIELD-CENTRIC <-> ROBOT-CENTRIC) is BACK on BOTH gamepads. The physical MODE
    // button cannot be used: the SDK has no field for it (see THE PHYSICAL "MODE" BUTTON in the header).

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

    // A Pinpoint that is "not READY" for LESS than this many seconds is treated as a GLITCH (one
    // bad I2C read, or a brief voltage dip), not a restart. During a glitch field-centric is NOT
    // switched off, and the hive memory is kept. If it is still not READY after this long, we
    // treat it as a restart: field-centric switches off and the hive memory is thrown away.
    // WHY: a Pinpoint that restarts forgets where it is (its heading is back at a new zero), so
    // a long gap must switch field-centric off. But a low battery can make it blink for a
    // moment, and we used to switch field-centric off for good every time it did.
    // Bigger = forgives longer gaps, but a real restart takes longer to be noticed.
    private static final double PINPOINT_GLITCH_TOLERANCE_SECONDS = 0.25;

    // When the Pinpoint comes back from a glitch, its heading must be within this many degrees of
    // the last heading we trusted. If it is further off, it probably RESTARTED and its heading is
    // back at a new zero, so field-centric switches off (the same as a long gap). A restart that
    // happens while the robot already faces about its INIT direction changes nothing, so it is
    // fine to miss. A driver spinning fast during a glitch can trip this too: that only means
    // field-centric switches off safely, and the driver's D-pad Left turns it back on.
    // Smaller = catches smaller restarts but trips more often. Bigger = the opposite.
    private static final double RESUME_MAX_HEADING_CHANGE_DEGREES = 20.0;

    // During a glitch (see PINPOINT_GLITCH_TOLERANCE_SECONDS), true = keep driving FIELD-CENTRIC
    // using the LAST good heading, so the sticks do not suddenly change meaning for a moment.
    // false = drive ROBOT-CENTRIC until the Pinpoint is READY again. The last heading can be a
    // little old if the robot is turning during the glitch, but that is only a fraction of a
    // second.
    private static final boolean HOLD_LAST_HEADING_DURING_GLITCH = true;

    // The target counts as "aimed at" when tx is within this many degrees of zero.
    private static final double AIMED_TOLERANCE_DEGREES = 2.0;

    // The camera must say "centered" (tx within AIMED_TOLERANCE_DEGREES) for this many seconds IN A ROW
    // before a volley may start. This is the "settled" check.
    // WHY NOT turretSubsystem.isOnTarget()? Auto-align picks a NEW target (angle - tx) every loop while
    // tx is over ALIGN_REAIM_THRESHOLD_DEGREES (1.0), and the camera's tx jitters a little, so every
    // pick clears the turret's own settle timer. The turret also stops pushing once it is within 1.5
    // degrees of its target. So it could rest at tx = 1.4 for ever: over the re-aim threshold (so it
    // kept picking new targets) but inside the turret's own band (so it never moved), and
    // isOnTarget() never became true. The volley sat in WAIT_FOR_READY while the turret looked locked
    // on (found on the robot, issue #63). The camera is the real truth about aim anyway.
    private static final double AIMED_HOLD_SECONDS = 0.15;

    // With NO camera target (DIAL mode only) there is no tx to check. The turret then counts as ready
    // when it is under our control (not floating) and within this many degrees of the angle it is
    // holding.
    private static final double NO_CAMERA_TURRET_READY_DEGREES = 3.0;

    // In TABLE mode, when the camera loses the hive, keep using the LAST table RPM for this many
    // seconds. After that the flywheel goes back to the dial. A short hold stops the wheels from
    // lurching if a tag drops out for a moment, and a limit stops a stale RPM from lasting.
    private static final double TABLE_RPM_HOLD_SECONDS = 1.0;

    // ShooterSubsystem treats ANY change of target RPM as "start over", and the table's answer
    // wobbles a little because ty does. So (outside a volley's feed and end, where the target
    // is frozen) we only change the flywheel target when the wanted RPM is more than this far from
    // the one we are already using. Otherwise the "at speed" check could keep restarting and the
    // feed would stutter. (The same number FiringSequenceAlpha uses.)
    private static final double RPM_CHANGE_DEADBAND = 10.0;

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
    private FiringSequence firingSequence;

    // PRE-REV (the driver's D-pad Up): are the wheels spinning even though nobody is firing?
    private boolean preRevOn = false;
    private boolean wasPreRevPressed = false;

    // SOLO MODE (gamepad1 LOGO): for a match with ONE person. false (the start) = only the OPERATOR's
    // Right Trigger fires. true = the driver's Right Trigger fires too.
    private boolean soloMode = false;
    private boolean wasSoloTogglePressed = false;

    // The flywheel RPM we are really using (the dial or the table, with the dead band applied).
    // While a volley feeds and ends, shotFrozen is true and the RPM and the ratio are held at
    // frozenFlywheelRpm and frozenRatio, so the target does not move while balls go through.
    private double commandedFlywheelRpm = 0.0;
    private boolean shotFrozen = false;
    private double frozenFlywheelRpm = 0.0;
    private double frozenRatio = 0.0;

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

    // For the "aimed" check: how long the camera has said "centered" in a row (see AIMED_HOLD_SECONDS).
    private ElapsedTime aimedTimer = new ElapsedTime();
    private boolean wasAimedNow = false;

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
    private boolean wasDriverStartPressed = false;
    private boolean wasSpeedSourceTogglePressed = false;

    // The push the turret got this loop to follow the robot's spin, for the screen.
    private double turretFeedforward = 0.0;

    // The Pinpoint's heading from the last loop, so we can spot a jump (see
    // MAX_PINPOINT_HEADING_JUMP_DEGREES). haveLastPinpointHeading is false until we have one.
    private double lastPinpointHeadingDegrees = 0.0;
    private boolean haveLastPinpointHeading = false;

    // The Pinpoint glitch debounce (see PINPOINT_GLITCH_TOLERANCE_SECONDS).
    // pinpointNotReadyTimer counts the seconds since the Pinpoint stopped being READY.
    // pinpointWasReadyLastLoop is true while it is READY, so the loop where it STOPS being READY
    // (and the loop where it comes BACK) can be spotted.
    private ElapsedTime pinpointNotReadyTimer = new ElapsedTime();
    private boolean pinpointWasReadyLastLoop = true;

    // For the screen: how many glitches the Pinpoint had that fixed themselves (it came back READY
    // and its heading was still right), and the longest one in seconds. Many glitches with a weak
    // battery is the sign of a voltage dip. Replace the battery first.
    private int pinpointGlitchCount = 0;
    private double pinpointLongestGlitchSeconds = 0.0;

    // ---- Field-centric drive ----
    // Does the driver team WANT field-centric? (The drive toggle switches it.) It starts ON.
    private boolean fieldCentricWanted = true;

    // true = the safety rule switched field-centric off. It stays off until the drive toggle
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

    private boolean wasOperatorDriveTogglePressed = false;
    private boolean wasDriverDriveTogglePressed = false;
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
        telemetry.addLine("Right Bumper = INTAKE   Left Bumper = REVERSE   (the intake is the driver's only)");
        telemetry.addLine("LOGO = SOLO MODE on / off (one-person match: then the driver's Right Trigger fires too)");
        telemetry.addLine("D-pad Up = PRE-REV on / off (wheels spin without firing)");
        telemetry.addLine("START = alliance BLUE / RED   BACK = drive FIELD-CENTRIC / ROBOT-CENTRIC");
        telemetry.addLine("D-pad Left = RE-AIM (the way the robot faces now is away from me)");
        telemetry.addLine("--- GAMEPAD 2 (OPERATOR) ---");
        telemetry.addLine("START = alliance BLUE / RED   BACK = drive FIELD-CENTRIC / ROBOT-CENTRIC");
        telemetry.addLine("LOGO = flywheel speed DIAL / TABLE");
        telemetry.addLine("D-pad Up / Down = flywheel RPM up / down");
        telemetry.addLine("D-pad Right / Left = ratio up / down");
        telemetry.addLine("X = gate CLOSED   B = gate OPENED   Y = gate LIFTED   (only between volleys)");
        telemetry.addLine("Right Trigger (hold) = FIRE a volley (let go = abort)");
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
        // The SDK guesses the type from how the controller introduces itself. Our Logitech controller can
        // show up as XBOX_360 in X mode. That is only the SDK's label, it is not an Xbox controller.
        telemetry.addData("Controller type (the SDK's guess, ours is a Logitech)", gamepad1.type());
        telemetry.addData("Left stick x, y", "%.2f, %.2f",
                gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Right stick x, y", "%.2f, %.2f",
                gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("Triggers L, R", "%.2f, %.2f",
                gamepad1.left_trigger, gamepad1.right_trigger);
    }

    /*
     * The names of the buttons a gamepad is holding down right now, in words a student can read, or
     * "none". Only buttons the SDK can read are here. The Logitech MODE button is NOT, because the
     * Gamepad class has no field for it.
     */
    private String getPressedButtonsText(Gamepad pad) {
        String text = "";
        if (pad.a) { text = text + "A "; }
        if (pad.b) { text = text + "B "; }
        if (pad.x) { text = text + "X "; }
        if (pad.y) { text = text + "Y "; }
        if (pad.dpad_up) { text = text + "D-UP "; }
        if (pad.dpad_down) { text = text + "D-DOWN "; }
        if (pad.dpad_left) { text = text + "D-LEFT "; }
        if (pad.dpad_right) { text = text + "D-RIGHT "; }
        if (pad.left_bumper) { text = text + "LB "; }
        if (pad.right_bumper) { text = text + "RB "; }
        if (pad.left_trigger > TRIGGER_PRESSED_THRESHOLD) { text = text + "LT "; }
        if (pad.right_trigger > TRIGGER_PRESSED_THRESHOLD) { text = text + "RT "; }
        if (pad.back) { text = text + "BACK "; }
        if (pad.start) { text = text + "START "; }
        if (pad.guide) { text = text + "LOGO "; }
        if (pad.left_stick_button) { text = text + "LEFT-STICK-CLICK "; }
        if (pad.right_stick_button) { text = text + "RIGHT-STICK-CLICK "; }
        if (text.isEmpty()) {
            return "none";
        }
        return text;
    }

    /*
     * The BUTTON TEST for the INIT screen: every button each gamepad reports, plus the left stick, so
     * you can see exactly what the Robot Controller gets. (The Logitech MODE button is a hardware
     * switch that swaps the D-pad and the left stick, and the code cannot read it. If its LED is on you
     * will see a D-pad press show up as stick movement and the other way round: press MODE again to
     * turn it OFF before you drive!) Shown only before START, so it does not crowd the screen while
     * driving.
     */
    private void addButtonTestTelemetry() {
        telemetry.addLine("--- BUTTON TEST (every button each gamepad reports. The physical MODE button is not readable: if its LED is on, press it again) ---");
        telemetry.addData("Gamepad1 buttons now", getPressedButtonsText(gamepad1));
        telemetry.addData("Gamepad1 left stick x, y", "%.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Gamepad2 buttons now", getPressedButtonsText(gamepad2));
        telemetry.addData("Gamepad2 left stick x, y", "%.2f, %.2f", gamepad2.left_stick_x, gamepad2.left_stick_y);
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
     * The driver's D-pad Up switches PRE-REV on and off. One switch per press. PRE-REV spins the
     * wheels (at the speed a shot would use) while you drive to the shooting spot, so the spin-up
     * is not paid at every volley. It does not touch the gate or the intake. It starts OFF.
     */
    private void checkPreRevToggle() {
        boolean preRevPressed = gamepad1.dpad_up;
        if (isNewPress(preRevPressed, wasPreRevPressed)) {
            preRevOn = !preRevOn;
        }
        wasPreRevPressed = preRevPressed;
    }

    /*
     * Gamepad1 LOGO (the center "guide" button) switches SOLO MODE on and off. One switch per press.
     * It works in INIT too, so it can be set before START. It changes nothing the robot does by
     * itself: it only lets the driver's Right Trigger fire a volley, for a match with one person. If
     * the Robot Controller never sees this button, solo mode just cannot be switched on, which is
     * safe (the operator can still fire).
     */
    private void checkSoloModeToggle() {
        boolean soloTogglePressed = gamepad1.guide;
        if (isNewPress(soloTogglePressed, wasSoloTogglePressed)) {
            soloMode = !soloMode;
        }
        wasSoloTogglePressed = soloTogglePressed;
    }

    /*
     * The words for "what is the volley still waiting for?", shown while a volley waits.
     */
    private String getWaitingText(boolean flywheelAtSpeed, boolean hasTarget, boolean aimedNow,
                                  boolean aimedAtHive, boolean turretHeld) {
        String waiting = "";
        if (!flywheelAtSpeed) {
            waiting = waiting + "flywheel not at speed   ";
        }
        if (!turretHeld) {
            waiting = waiting + "turret is floating (gamepad2 A is held)   ";
        } else if (hasTarget) {
            if (!aimedNow) {
                waiting = waiting + String.format("turret not aimed yet (the camera says %.1f deg off, it needs %.1f or less)   ",
                        Math.abs(visionSubsystem.getTxDegrees()), AIMED_TOLERANCE_DEGREES);
            } else if (!aimedAtHive) {
                waiting = waiting + "camera is centered, making sure it stays that way   ";
            }
        } else if (tableMode) {
            waiting = waiting + "NO TARGET (the camera must see the hive; gamepad2 LOGO switches to DIAL for a shot without the camera)   ";
        } else {
            double turretErrorDegrees = Math.abs(turretSubsystem.getTargetAngleDegrees() - turretSubsystem.getAngleDegrees());
            if (turretErrorDegrees > NO_CAMERA_TURRET_READY_DEGREES) {
                waiting = waiting + String.format("turret not settled (%.1f deg from the angle it is holding)   ", turretErrorDegrees);
            }
        }
        if (waiting.isEmpty()) {
            return "nothing, ready to fire";
        }
        return waiting;
    }

    /*
     * START on either gamepad switches the alliance (BLUE <-> RED), and gamepad2 LOGO switches where
     * the flywheel speed comes from (DIAL <-> TABLE). One switch per press. This works
     * before and after START, so the operator can set both during INIT.
     */
    private void checkModeButtons() {
        boolean startPressed = gamepad2.start;
        boolean driverStartPressed = gamepad1.start;
        boolean speedSourceTogglePressed = gamepad2.guide;   // gamepad2 LOGO

        if (isNewPress(speedSourceTogglePressed, wasSpeedSourceTogglePressed)) {
            tableMode = !tableMode;
        }
        // START on EITHER gamepad switches the alliance. Each gamepad is watched on its own, so
        // holding one START down never hides a press on the other.
        if (isNewPress(startPressed, wasStartPressed)
                || isNewPress(driverStartPressed, wasDriverStartPressed)) {
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
        wasDriverStartPressed = driverStartPressed;
        wasSpeedSourceTogglePressed = speedSourceTogglePressed;
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
     * switches field-centric OFF. Turning it back on is up to the drive toggle or the
     * driver's D-pad Left.
     *   - Pinpoint not READY for LONGER than PINPOINT_GLITCH_TOLERANCE_SECONDS: it is missing, still
     *     calibrating, faulted, or it restarted. A shorter gap is a GLITCH (see below).
     *   - Heading jumped: it changed by more than a robot can turn in one loop, so the Pinpoint
     *     restarted or glitched (a power dip can do this). Its heading is back at some new zero,
     *     so the remembered direction, which was measured against the OLD zero, is now wrong.
     *
     * GLITCH DEBOUNCE: a Pinpoint that is not READY for a moment (one bad read, a brief voltage
     * dip) used to switch field-centric off for good. Now a gap shorter than
     * PINPOINT_GLITCH_TOLERANCE_SECONDS changes nothing: the drive keeps the last good heading
     * (HOLD_LAST_HEADING_DURING_GLITCH), the hive memory is kept (the turret just holds until the
     * Pinpoint is back, see aimWithoutCamera), and when the Pinpoint is READY again its heading
     * must be within RESUME_MAX_HEADING_CHANGE_DEGREES of the last good one. If it is, the glitch
     * is counted on the screen and everything carries on. If not, it probably restarted, so we
     * switch field-centric off like before.
     */
    private void checkPinpointHealth() {
        if (!pinpointSubsystem.isReady()) {
            if (pinpointWasReadyLastLoop) {
                // The gap just started. Count the seconds from now.
                pinpointNotReadyTimer.reset();
                pinpointWasReadyLastLoop = false;
            }
            if (pinpointNotReadyTimer.seconds() > PINPOINT_GLITCH_TOLERANCE_SECONDS) {
                // Too long to be a glitch, so treat it as a restart.
                hiveMemory.clear();
                haveLastPinpointHeading = false;
                fieldCentricBlocked = true;
                fieldCentricBlockedReason = String.format("the Pinpoint was not READY for more than %.2f seconds",
                        PINPOINT_GLITCH_TOLERANCE_SECONDS);
            }
            return;
        }

        // The Pinpoint is READY. Did it just come back from a gap?
        boolean justCameBack = !pinpointWasReadyLastLoop;
        double gapSeconds = pinpointNotReadyTimer.seconds();
        pinpointWasReadyLastLoop = true;

        double heading = pinpointSubsystem.getHeadingDegrees();
        if (haveLastPinpointHeading) {
            double change = Math.abs(HiveBearingMemory.wrapDegrees(heading - lastPinpointHeadingDegrees));
            if (justCameBack) {
                // A glitch lasts several loops, so after one the heading is allowed to have moved a
                // little. Check against the (smaller) resume limit.
                if (change > RESUME_MAX_HEADING_CHANGE_DEGREES) {
                    hiveMemory.clear();
                    fieldCentricBlocked = true;
                    fieldCentricBlockedReason = String.format(
                            "the Pinpoint came back from a glitch with its heading %.0f degrees off (it probably restarted)",
                            change);
                } else {
                    pinpointGlitchCount = pinpointGlitchCount + 1;
                    if (gapSeconds > pinpointLongestGlitchSeconds) {
                        pinpointLongestGlitchSeconds = gapSeconds;
                    }
                }
            } else if (change > MAX_PINPOINT_HEADING_JUMP_DEGREES) {
                hiveMemory.clear();
                fieldCentricBlocked = true;
                fieldCentricBlockedReason = String.format("the Pinpoint heading jumped %.0f degrees", change);
            }
        }
        lastPinpointHeadingDegrees = heading;
        haveLastPinpointHeading = true;
    }

    /*
     * True while the Pinpoint is not READY but the gap is still short enough to be a glitch, and
     * we have a good heading to hold. (Only used when HOLD_LAST_HEADING_DURING_GLITCH is true.)
     */
    private boolean isPinpointInGlitch() {
        return HOLD_LAST_HEADING_DURING_GLITCH
                && !pinpointSubsystem.isReady()
                && haveLastPinpointHeading;
    }

    /*
     * The Pinpoint heading the field-centric drive uses. Normally the live one. During a glitch
     * it is the last good one.
     */
    private double getPinpointHeadingForDrive() {
        if (pinpointSubsystem.isReady()) {
            return pinpointSubsystem.getHeadingDegrees();
        }
        return lastPinpointHeadingDegrees;
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
     * True when the driver is pressing the DRIVE TOGGLE button on gamepad1. It is BACK.
     *
     * WHY NOT THE PHYSICAL "MODE" BUTTON: the coach asked for it, but the FTC SDK's Gamepad class has
     * no field for the Logitech MODE button (it is a hardware switch that swaps the D-pad and left
     * stick). If the INIT screen's BUTTON TEST ever shows a button that changes when MODE is pressed,
     * point these two methods at it. One line each.
     */
    private boolean isDriveToggleButtonPressedOnDriverPad() {
        return gamepad1.back;
    }

    // True when the operator is pressing the DRIVE TOGGLE button on gamepad2. It is BACK too.
    private boolean isDriveToggleButtonPressedOnOperatorPad() {
        return gamepad2.back;
    }

    /*
     * The drive toggle (BACK on either gamepad) and the driver's D-pad Left. Call it once
     * per loop.
     *   TOGGLE: if the safety rule switched field-centric off, turn it back ON. Otherwise switch
     *         between FIELD-CENTRIC and ROBOT-CENTRIC.
     *   D-pad Left (RE-AIM): "the way the robot faces right now is away from me". It only works
     *         while the Pinpoint is READY (it needs the heading). The Pinpoint is NOT reset. It also
     *         turns field-centric back on if the safety rule had switched it off.
     */
    private void checkDriveModeButtons() {
        boolean operatorTogglePressed = isDriveToggleButtonPressedOnOperatorPad();
        boolean driverTogglePressed = isDriveToggleButtonPressedOnDriverPad();
        boolean reAimPressed = gamepad1.dpad_left;

        // Each gamepad is watched on its own, so holding one down never hides a press on the other.
        if (isNewPress(operatorTogglePressed, wasOperatorDriveTogglePressed)
                || isNewPress(driverTogglePressed, wasDriverDriveTogglePressed)) {
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

        wasOperatorDriveTogglePressed = operatorTogglePressed;
        wasDriverDriveTogglePressed = driverTogglePressed;
        wasReAimPressed = reAimPressed;
    }

    /*
     * Is field-centric being used right now? The operator wants it, the safety rule has not
     * switched it off, and the Pinpoint is READY (or only in a short glitch, see
     * isPinpointInGlitch, with the last good heading to hold).
     */
    private boolean isFieldCentricActive() {
        return fieldCentricWanted && !fieldCentricBlocked
                && (pinpointSubsystem.isReady() || isPinpointInGlitch());
    }

    /*
     * The words for the "DRIVE MODE" line on the screen.
     */
    private String getDriveModeText() {
        if (!fieldCentricWanted) {
            return "ROBOT-CENTRIC (BACK on either gamepad switches to FIELD-CENTRIC)";
        }
        if (fieldCentricBlocked) {
            return "ROBOT-CENTRIC: field-centric was switched off because " + fieldCentricBlockedReason
                    + ". BACK on either gamepad or the driver's D-pad Left turns it on again.";
        }
        if (isPinpointInGlitch()) {
            return String.format("FIELD-CENTRIC (the Pinpoint is not READY: holding the last heading, %.2f of %.2f s)",
                    pinpointNotReadyTimer.seconds(), PINPOINT_GLITCH_TOLERANCE_SECONDS);
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
        firingSequence = new FiringSequence();

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
        // The drive toggle and D-pad Left already work here, so you can try them before START.
        updateDriveFrameFromAlliance();
        checkDriveModeButtons();
        checkSoloModeToggle();

        // The Driver Station clears its screen every loop, so we add the
        // button map again here. That keeps it on the screen until START.
        telemetry.addData("Status", "Ready to start!");
        telemetry.addData("Alliance (START on either gamepad)", visionSubsystem.getAlliance());
        telemetry.addData("DRIVE MODE (BACK on either gamepad)", getDriveModeText());
        telemetry.addData("SOLO MODE (gamepad1 LOGO)", soloMode ? "ON  (the driver's Right Trigger fires too)" : "off (only the operator fires)");
        telemetry.addData("Flywheel speed source (gamepad2 LOGO)", getModeText());
        visionSubsystem.addTelemetry(telemetry);
        // Check that the Pinpoint says READY before START. (The robot must be still until then.)
        pinpointSubsystem.addTelemetry(telemetry);
        addButtonTestTelemetry();
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
        checkPreRevToggle();
        checkSoloModeToggle();

        // ---- Gamepad 1 (driver) ----
        // On the controller, pushing a stick FORWARD gives a negative number, and pushing it RIGHT
        // gives a positive number. Our subsystem wants forward +, LEFT +, and COUNTERCLOCKWISE +,
        // so we flip the sign of all three. (Turning right is clockwise, which is negative.)
        double forward = applyDeadband(-gamepad1.left_stick_y);
        double strafe = applyDeadband(-gamepad1.left_stick_x);
        double turn = applyDeadband(-gamepad1.right_stick_x);

        boolean slowMode = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;
        // WHO FIRES (see NEW IN ALPHA 13): the OPERATOR's Right Trigger (gamepad2), held, fires a
        // volley. The driver's Right Trigger fires too ONLY in SOLO MODE (gamepad1 LOGO).
        boolean operatorFireHeld = gamepad2.right_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean driverFireHeld = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean fireHeld = operatorFireHeld || (soloMode && driverFireHeld);

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

        // The FIRING SEQUENCE freezes the shot while balls go through (see NEW IN ALPHA 13): during the
        // feed and the end of a volley the flywheel RPM AND the ratio are held where they
        // were, so the target does not move under the balls. In the other steps the target only
        // changes when the wanted RPM moves by more than RPM_CHANGE_DEADBAND, so a wobbling table
        // answer cannot keep restarting ShooterSubsystem's "at speed" check. (The sequence's step is
        // from the last loop. It changes only in step 3 below.)
        FiringSequence.SequenceState sequenceStep = firingSequence.getState();
        boolean feedingOrLater = sequenceStep == FiringSequence.SequenceState.FEED
                || sequenceStep == FiringSequence.SequenceState.DONE;
        double shotRatio = ratio;
        if (feedingOrLater) {
            if (!shotFrozen) {
                frozenFlywheelRpm = commandedFlywheelRpm;
                frozenRatio = ratio;
                shotFrozen = true;
            }
            flywheelTargetRpm = frozenFlywheelRpm;
            shotRatio = frozenRatio;
        } else {
            shotFrozen = false;
            if (commandedFlywheelRpm == 0.0
                    || Math.abs(flywheelTargetRpm - commandedFlywheelRpm) > RPM_CHANGE_DEADBAND) {
                commandedFlywheelRpm = flywheelTargetRpm;
            }
            flywheelTargetRpm = commandedFlywheelRpm;
        }

        double wantedHoodWheelRpm = flywheelTargetRpm * shotRatio;
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
            // (During a short Pinpoint glitch this is the last good heading, not the live one.)
            drivingHeadingDegrees = getPinpointHeadingForDrive() + fieldHeadingOffsetDegrees;
            driveSubsystem.driveFieldCentric(forward, strafe, turn, drivingHeadingDegrees);
            driveControl = driveControl + ", FIELD-CENTRIC";
        } else {
            driveSubsystem.drive(forward, strafe, turn);
            driveControl = driveControl + ", ROBOT-CENTRIC";
        }

        // Shooter wheels. They spin while the driver HOLDS the Right Trigger (to fire) or while PRE-REV
        // is ON, at the same speed either way: the DIAL or the TABLE, frozen during a volley (see
        // above). (There is no wheel reverse in this OpMode any more.)
        boolean wheelsWanted = fireHeld || preRevOn;
        String shooterControl;
        if (wheelsWanted) {
            // The shooter's speed control holds each wheel at its target, even when a ball slows it
            // down or the battery drains.
            shooterSubsystem.setTargetRpms(flywheelTargetRpm, hoodWheelTargetRpm);
            if (fireHeld && operatorFireHeld) {
                shooterControl = "Operator Right Trigger (FIRE)";
            } else if (fireHeld) {
                shooterControl = "Driver Right Trigger (FIRE, SOLO MODE)";
            } else {
                shooterControl = "PRE-REV";
            }
        } else {
            // Not firing and not pre-revving, so both wheels get 0 power and coast down.
            shooterSubsystem.stop();
            shooterControl = "OFF";
        }
        shooterSubsystem.update();

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

        // The FIRING SEQUENCE. It comes AFTER the shooter and turret updates, so "flywheel at speed"
        // and the turret's answers are fresh. READY TO START means: the flywheel is at speed (the
        // flywheel ONLY, like ShootAndParkAuto) AND the turret's half is true:
        //   - The turret must be under our control. If it is floating (gamepad2 A held), a volley waits.
        //   - If the camera sees the hive (either speed source), the CAMERA decides: tx must be within
        //     AIMED_TOLERANCE_DEGREES for AIMED_HOLD_SECONDS in a row. (We do not use
        //     turretSubsystem.isOnTarget(): see the note at AIMED_HOLD_SECONDS.)
        //   - No camera target in TABLE mode: not ready (TABLE needs the camera).
        //   - No camera target in DIAL mode: the turret just has to be within
        //     NO_CAMERA_TURRET_READY_DEGREES of the angle it is holding.
        boolean flywheelAtSpeed = shooterSubsystem.isFlywheelAtSpeed();
        boolean turretHeld = turretSubsystem.getState() == TurretSubsystem.TurretState.POSITION;
        boolean aimedNow = hasTarget
                && Math.abs(visionSubsystem.getTxDegrees()) <= AIMED_TOLERANCE_DEGREES;
        if (aimedNow && !wasAimedNow) {
            aimedTimer.reset();
        }
        wasAimedNow = aimedNow;
        boolean aimedAtHive = aimedNow && aimedTimer.seconds() >= AIMED_HOLD_SECONDS;
        boolean turretReady;
        if (!turretHeld) {
            turretReady = false;
        } else if (hasTarget) {
            turretReady = aimedAtHive;
        } else if (tableMode) {
            turretReady = false;
        } else {
            double turretErrorDegrees = Math.abs(turretSubsystem.getTargetAngleDegrees() - turretSubsystem.getAngleDegrees());
            turretReady = turretErrorDegrees <= NO_CAMERA_TURRET_READY_DEGREES;
        }
        boolean readyToStart = flywheelAtSpeed && turretReady;
        firingSequence.update(fireHeld, readyToStart, flywheelAtSpeed);

        // Shooter gate. CLOSING is always GENTLE (goToClosedGently): the gate walks toward CLOSED slowly,
        // so a single ball left in the way is pushed aside instead of slammed or wedged. OPENING and
        // LIFTING are instant. (shooterServoSubsystem.update() below walks the gentle close along.)
        // While a volley runs the FIRING SEQUENCE is in charge. Between volleys the
        // operator's buttons work: X = CLOSED, B = OPENED, Y = LIFTED. A button counts as ONE press
        // only on the loop where it goes from "not pressed" to "pressed", so holding it does not keep
        // re-sending the move. We work the presses out every loop, even during a volley, so an old
        // press never waits around. If more than one is pressed on the same loop, CLOSED wins, then
        // OPENED, then LIFTED.
        boolean closedNewPress = isNewPress(gateClosedPressed, wasGateClosedPressed);
        boolean openedNewPress = isNewPress(gateOpenedPressed, wasGateOpenedPressed);
        boolean liftedNewPress = isNewPress(gateLiftedPressed, wasGateLiftedPressed);
        FiringSequence.GateCommand gateCommand = firingSequence.getGateCommand();
        String gateControl;
        if (gateCommand == FiringSequence.GateCommand.CLOSE) {
            shooterServoSubsystem.goToClosedGently();
            gateControl = "firing sequence (CLOSED, gently)";
        } else if (gateCommand == FiringSequence.GateCommand.OPEN) {
            shooterServoSubsystem.goToOpened();
            gateControl = "firing sequence (OPENED)";
        } else if (closedNewPress) {
            shooterServoSubsystem.goToClosedGently();
            gateControl = "manual (X, gently)";
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

        // Intake. It is the DRIVER's only (gamepad1 bumpers: the operator's bumpers do nothing).
        //   1. REVERSE (Left Bumper) goes first, so clearing a jam always works.
        //   2. GATE CLOSING: any time the gate is walking toward CLOSED (from OPENED, or from LIFTED),
        //      the intake runs in REVERSE for exactly that long, on its own. A ball that is in the
        //      gate's way is pushed back out of it instead of being pulled in against the closing
        //      gate and wedged. This beats the driver's INTAKE button (pulling balls in while the gate
        //      shuts is what wedges them), and it ends the loop the gate arrives at CLOSED.
        //      (isClosingGently() was brought up to date by shooterServoSubsystem.update() just above.)
        //   3. Then the firing sequence, while it is feeding. (It cannot overlap with 2: the gate is
        //      OPENED while it feeds, and opening ends a gentle close.)
        //   4. Then INTAKE (Right Bumper).
        String intakeControl;
        if (gamepad1.left_bumper) {
            intakeSubsystem.reverse();
            intakeControl = "Left Bumper (REVERSE)";
        } else if (shooterServoSubsystem.isClosingGently()) {
            intakeSubsystem.reverse();
            intakeControl = "gate closing: REVERSE (automatic)";
        } else if (firingSequence.ownsIntake()) {
            if (firingSequence.wantsIntake()) {
                intakeSubsystem.intake();
                intakeControl = "firing sequence: FEEDING";
            } else {
                intakeSubsystem.stop();
                intakeControl = "firing sequence: waiting for the flywheel";
            }
        } else if (gamepad1.right_bumper) {
            intakeSubsystem.intake();
            intakeControl = "Right Bumper (INTAKE)";
        } else {
            // No bumper held, so stop the intake.
            intakeControl = "None";
            intakeSubsystem.stop();
        }

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
        telemetry.addData("Alliance (START on either gamepad)", visionSubsystem.getAlliance());
        telemetry.addData("DRIVE MODE (BACK on either gamepad)", getDriveModeText());
        if (soloMode) {
            telemetry.addData("SOLO MODE (gamepad1 LOGO)", "ON  (the driver's Right Trigger fires too)");
        } else {
            telemetry.addData("SOLO MODE (gamepad1 LOGO)", "off (only the operator's Right Trigger fires)");
        }
        if (preRevOn) {
            telemetry.addData("PRE-REV (driver D-pad Up)", "ON  (the wheels are spinning)");
        } else {
            telemetry.addData("PRE-REV (driver D-pad Up)", "off");
        }

        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive Control", driveControl);
        if (fieldCentricActive) {
            telemetry.addData("Heading the drive uses (deg)", "%.1f   (Pinpoint %.1f + offset %.1f)",
                    drivingHeadingDegrees, getPinpointHeadingForDrive(), fieldHeadingOffsetDegrees);
        }
        // How often the Pinpoint blinked and came back by itself. Many of these with a weak battery
        // means voltage dips. If the drive keeps going ROBOT-CENTRIC, this and the reason in the
        // DRIVE MODE line say whether the Pinpoint restarted.
        telemetry.addData("Pinpoint glitches (came back by themselves)", "%d   longest %.2f s",
                pinpointGlitchCount, pinpointLongestGlitchSeconds);
        telemetry.addData("Camera pipeline fixes", "%d   (above 0 = the camera restarted or lost its pipeline)",
                visionSubsystem.getPipelineFixCount());
        telemetry.addData("Drive inputs", "forward %.2f   strafe %.2f   turn %.2f",
                forward, strafe, turn);

        // The Pinpoint prints its own "--- ODOMETRY (Pinpoint) ---" heading, with its status, the
        // robot's position (X, Y, heading) and its speeds. Nothing uses the position yet.
        pinpointSubsystem.addTelemetry(telemetry);

        // The dials: what the operator has asked for. (They take effect while the driver holds the
        // Right Trigger, or while PRE-REV is ON.)
        telemetry.addLine("--- SHOOTER DIALS (RPM) ---");
        telemetry.addData("Flywheel speed source (gamepad2 LOGO)", getModeText());
        telemetry.addData("Using", rpmSource);
        if (flywheelCapped) {
            telemetry.addData("Flywheel target RPM", "%.0f  CAPPED at %.0f (you asked for %.0f)",
                    maxFlywheelRpm, maxFlywheelRpm, flywheelTargetRpm);
        } else {
            telemetry.addData("Flywheel target RPM", "%.0f   (the dial is at %d)",
                    flywheelTargetRpm, flywheelRpmDial);
        }
        if (shotFrozen) {
            telemetry.addData("Ratio (hood / flywheel)", "%.2f   (frozen during the volley, the dial is at %.2f)", shotRatio, ratio);
        } else {
            telemetry.addData("Ratio (hood / flywheel)", "%.2f", shotRatio);
        }
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
        telemetry.addData("OK to shoot? (both wheels, for information)", readyText);
        // What the firing sequence really checks: the flywheel only. The hood wheel's error is shown
        // so we can see whether "close enough" is true (issue #62).
        telemetry.addData("Flywheel at speed (the volley's check)", flywheelAtSpeed);
        telemetry.addData("Hood wheel error (actual - target, RPM)", "%.0f",
                shooterSubsystem.getHoodWheelRpm() - shooterSubsystem.getTargetHoodWheelRpm());

        // The hood wheel RPM / flywheel RPM we really have right now. It should match the ratio
        // dial once the wheels are at speed. If it does not, the hood wheel may be CAPPED.
        if (shooterSubsystem.getFlywheelRpm() > MIN_RPM_FOR_MEASURED_RATIO) {
            telemetry.addData("Ratio measured now", "%.2f",
                    shooterSubsystem.getHoodWheelRpm() / shooterSubsystem.getFlywheelRpm());
        }

        // The gate prints its own "--- SHOOTER GATE ---" heading.
        shooterServoSubsystem.addTelemetry(telemetry);
        telemetry.addData("Gate control", gateControl);

        // The firing sequence prints its own "--- FIRING SEQUENCE ---" heading and the last volley's
        // log. While a volley waits, say what it is waiting for.
        firingSequence.addTelemetry(telemetry);
        if (firingSequence.getState() == FiringSequence.SequenceState.WAIT_FOR_READY) {
            telemetry.addData("Waiting for", getWaitingText(flywheelAtSpeed, hasTarget, aimedNow, aimedAtHive, turretHeld));
        }

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

        // Is the turret's half of the firing gate open? The camera must say we are centered on the
        // hive (within AIMED_TOLERANCE_DEGREES) for AIMED_HOLD_SECONDS in a row.
        String aimText;
        if (!hasTarget) {
            aimText = "NO TARGET";
        } else if (aimedAtHive) {
            aimText = String.format("YES - the camera says we are aimed (%.1f deg off)", Math.abs(visionSubsystem.getTxDegrees()));
        } else if (aimedNow) {
            aimText = "almost - the camera is centered, making sure it stays that way";
        } else {
            aimText = String.format("NO - off target by %.1f deg (tx), it needs %.1f or less",
                    visionSubsystem.getTxDegrees(), AIMED_TOLERANCE_DEGREES);
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
        // Always leave the motors stopped. Close the gate too, so pressing STOP in the middle of a
        // volley cannot leave it OPENED or LIFTED. (A servo holds its position by itself.)
        shooterServoSubsystem.goToClosed();
        driveSubsystem.stop();
        intakeSubsystem.stop();
        shooterSubsystem.stop();
        turretSubsystem.stop();
        // Stop polling the camera.
        visionSubsystem.stop();
        // The Pinpoint only reads, so there is nothing to stop.
    }
}
