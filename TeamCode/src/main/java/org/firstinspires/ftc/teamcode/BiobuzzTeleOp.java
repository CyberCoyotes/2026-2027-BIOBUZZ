package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * TeleOp Alpha 7
 * An iterative OpMode with TWO controllers:
 *   gamepad1 = the DRIVER: drives the robot, runs the INTAKE, and holds the trigger to SHOOT.
 *              (The turret now AUTO-TRACKS the hive ALL THE TIME, see ALWAYS-TRACK below.)
 *   gamepad2 = the OPERATOR: sets the shooter speeds (RPM), aims the TURRET, and moves the GATE.
 * It is DriveAlpha, IntakeAlpha, ShooterRPMAlpha, TurretAlpha, VisionAlpha and ShotTableAlpha
 * put together, so each part has already been tested on its own. This OpMode owns NO hardware.
 * It only reads the gamepads and tells the subsystems what to do.
 *
 * GAMEPAD 2 (the operator) has NO layers. Every button always does the same thing:
 *   Left Stick X     -> turn the turret (push RIGHT and the turret turns RIGHT)
 *   Y -> turret CENTER (0 degrees)   X -> FAR LEFT (+90)   B -> FAR RIGHT (-90)
 *   A -> STOP the turret (it floats, so you can turn it by hand)
 *   D-pad Up / Down  -> flywheel target RPM +/- FLYWHEEL_RPM_STEP
 *   D-pad Right / Left -> ratio (hood wheel / flywheel) +/- 0.05
 *   Left Bumper      -> HOLD to reverse BOTH shooter wheels (clearing a jam)
 *   Right Bumper     -> HOLD to reverse the flywheel only
 *   Right Stick Button (press the stick in) -> switch ALWAYS-TRACK ON <-> OFF (starts ON).
 *                       See ALWAYS-TRACK below. The driver's Right Trigger still aligns while
 *                       it shoots, even if ALWAYS-TRACK is OFF.
 *   Start            -> switch the flywheel speed source DIAL <-> TABLE (starts as DIAL)
 *   Back             -> switch alliance BLUE <-> RED (starts as BLUE). Check it on the screen!
 *   Right Trigger    -> shooter GATE to OPENED (lets the ball through)
 *   Left Trigger     -> shooter GATE to CLOSED (holds the ball back)
 *      (One move per press. The gate positions are CLOSED_POSITION, OPENED_POSITION and
 *       LIFTED_POSITION in ShooterServoSubsystem. If CLOSED and another move are pressed at
 *       the same time, CLOSED wins. The operator's triggers have no LIFTED button, only the
 *       driver's Y does.)
 *   With ALWAYS-TRACK ON, letting go of the stick goes back to tracking the hive. With it OFF,
 *   the turret holds where it is. A preset (Y, X, B) goes to its angle and holds, and so does
 *   A (STOP), because pressing any of them turns ALWAYS-TRACK OFF. Press the Right Stick Button
 *   to turn it back ON. The turret will not go past +/-120 degrees (see TurretSubsystem). 0 degrees is where
 *   the turret pointed when the Control Hub powered on, because the home switch is not
 *   installed yet, so power on with the turret pointing straight ahead.
 *
 * It starts ROBOT-CENTRIC: "forward" means the way the robot is facing.
 *
 * GAMEPAD 1 - DRIVER
 *   Left Stick   -> drive forward / backward and slide (strafe) left / right
 *   Right Stick  -> turn (push right to turn right)
 *   HOLD the Left Trigger -> SLOW MODE for careful lining up
 *   Right Bumper -> INTAKE (pull balls in)      Left Bumper -> REVERSE (push balls out)
 *      (Left Bumper beats Right Bumper, so clearing a jam always works. The intake runs at the
 *       power set in IntakeSubsystem, DEFAULT_INTAKE_POWER.)
 *   Right Trigger -> HOLD to SHOOT and AUTO-ALIGN the turret on the hive TOGETHER.
 *      The wheels run at the flywheel speed from the DIAL or the TABLE (see THE SHOT TABLE
 *      below), and the hood wheel at that x the ratio dial. At the start that is DIAL mode,
 *      flywheel 1300 RPM and hood wheel 2600 RPM (a ratio of 2.00).
 *      Let go and both wheels go to 0 power and coast down. The turret holds where it ended.
 *      Reverse (gamepad2 bumpers) still beats the wheels, so clearing a jam always works.
 *      The operator's turret stick and A (STOP) still beat the auto-align.
 *   X / B / Y    -> shooter GATE by hand, for testing: X = CLOSED, B = OPENED, Y = LIFTED.
 *      (One move per press. These are for manual control and testing. They work at the same
 *       time as the operator's gate triggers below, and the latest press wins. Later the gate
 *       will become part of a smart shooting sequence.)
 *
 * GAMEPAD 2 - OPERATOR, SHOOTER DIALS (SPEED CONTROL in RPM, read from the wheel encoders)
 *   D-pad Up / Down    -> flywheel target RPM UP / DOWN by FLYWHEEL_RPM_STEP
 *   D-pad Right / Left -> ratio UP / DOWN by 0.05
 *   Left Bumper  -> HOLD to reverse BOTH the flywheel and the hood wheel (clearing a jam)
 *   Right Bumper -> HOLD to reverse the flywheel only
 *   The hood wheel target RPM is flywheel RPM x ratio (RATIO = HOOD WHEEL / FLYWHEEL). The
 *   shooter will not go past each wheel's own maximum (the flywheel motor is the 1620 RPM one,
 *   so its maximum is only 1458 RPM). If a wheel would be over its maximum, the screen says
 *   CAPPED and the real ratio is lower than the dial says.
 *   Jam clearing (the bumpers) still uses plain negative POWER, because speed control only
 *   works forward.
 *   The screen shows "OK to shoot" only when BOTH wheels are really at their target RPM
 *   (the shooter's isAtSpeed()). It drops back to WAIT after a ball slows the wheels.
 *   ShooterRPMAlpha has the same wheel speed control on its own, for testing.
 *
 * THE SHOT TABLE (TABLE mode, switched with gamepad2 Start)
 *   The Limelight is on the turret. It reads ty, how far UP the hive's AprilTags appear in its
 *   view, and ShotTable turns that into a flywheel RPM. (Closer = bigger ty = lower RPM.)
 *   - DIAL mode: the flywheel uses the operator's dial, exactly like before. This is the safe
 *     fallback that needs no camera.
 *   - TABLE mode: while the camera sees the hive, the flywheel uses the table's RPM. If the
 *     camera loses the hive, it keeps the LAST table RPM for TABLE_RPM_HOLD_SECONDS so the wheels
 *     do not lurch, then goes back to the dial. The screen says which one is being used.
 *   - The camera can only see the hive when the turret points at it. ALWAYS-TRACK (below) keeps
 *     the turret pointed at it, and the driver's Right Trigger aligns it too while it shoots.
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
 * ALWAYS-TRACK (new in Alpha 7)
 *   Before, the turret only aligned on the hive while a button was held. Now it aligns on the
 *   hive ALL THE TIME, the whole match, with no button held. It uses the same auto-align as the
 *   trigger (see AUTO-ALIGN above), including following the robot's spin.
 *   - It starts ON. It does NOT run during INIT, because the robot must not move before START.
 *   - If the camera cannot see the hive, the turret HOLDS where it is. It does not search.
 *   - The operator's turret stick beats it while the stick is pushed. Let go of the stick and
 *     it goes back to tracking.
 *   - Y, X, B (presets) and A (STOP) switch it OFF so the turret stays where they put it.
 *     Press the Right Stick Button to switch it back ON. The screen shows ON or OFF.
 *   - The ALLIANCE (gamepad2 Back) must be right, or the turret tracks the OTHER team's hive.
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
 */
@TeleOp(name = "TeleOp - Alpha 7", group = "Testing")
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

    // How many percent make a whole power of 1.0 (100 percent = 1.0 power).
    // The turret power below is still counted in percent.
    private static final double PERCENT_PER_ONE = 100.0;

    // A hood wheel target counts as CAPPED when it is higher than the shooter's maximum by more
    // than this many RPM. (A little room, so a tiny rounding error does not cry wolf.)
    private static final double CAPPED_TOLERANCE_RPM = 1.0;

    // The measured ratio (hood RPM / flywheel RPM) is only worth showing when the flywheel is
    // spinning faster than this. Below it the division gives silly numbers.
    private static final double MIN_RPM_FOR_MEASURED_RATIO = 300.0;

    // Reverse power is NEGATIVE so the motor spins the other way. Jam clearing is plain power,
    // not RPM, because speed control in ShooterSubsystem only goes forward.
    private static final double FLYWHEEL_POWER_REVERSE = -0.60;
    private static final double HOOD_WHEEL_POWER_REVERSE = -0.60;

    // ------------------------------------------------------------------
    // Turret (gamepad2). These are test-only numbers, the same as in TurretAlpha.
    // ------------------------------------------------------------------
    // The turret power, in percent. A full push on the stick turns the turret at this power.
    // (D-pad Up / Down used to change it. They now change the flywheel RPM instead.)
    private static final int TURRET_POWER_PERCENT = 100;

    // The turret preset angles. LEFT is POSITIVE and RIGHT is NEGATIVE, the same as Pinpoint
    // and Pedro Pathing. Center is 0 degrees, which is 0 encoder ticks.
    private static final double TURRET_PRESET_SIDE_DEGREES = 90.0;
    private static final double TURRET_PRESET_LEFT_DEGREES = TURRET_PRESET_SIDE_DEGREES;
    private static final double TURRET_PRESET_RIGHT_DEGREES = -TURRET_PRESET_SIDE_DEGREES;
    private static final double TURRET_PRESET_CENTER_DEGREES = 0.0;

    // The turret stick must move more than this before it counts.
    private static final double TURRET_STICK_DEADBAND = 0.1;

    // ------------------------------------------------------------------
    // Vision and the shot table. These are test-only numbers, the same as in VisionAlpha.
    // ------------------------------------------------------------------
    // Auto-align picks a new turret target only when the camera's tx is bigger than this many
    // degrees. Smaller = more precise but twitchier. The hive window forgives about 5 degrees
    // or more, so there is no need to chase every fraction of a degree.
    private static final double ALIGN_REAIM_THRESHOLD_DEGREES = 1.0;

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

    // Should the turret track the hive ALL THE TIME from START? true = yes (ALWAYS-TRACK starts
    // ON), false = it starts OFF and the operator turns it on with the Right Stick Button.
    // Either way it never moves during INIT.
    private static final boolean ALWAYS_TRACK_AT_START = true;

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
    private boolean wasGateShootTriggerPressed = false;
    private boolean wasGateLoadTriggerPressed = false;
    private boolean wasDriverGateClosedPressed = false;
    private boolean wasDriverGateOpenedPressed = false;
    private boolean wasDriverGateLiftedPressed = false;
    private boolean wasDpadLeftPressed = false;
    private boolean wasDpadRightPressed = false;
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;
    private boolean wasStartPressed = false;
    private boolean wasBackPressed = false;

    // Remembers whether the turret stick was turning the turret last loop, so we can
    // hold the angle when it is released. The same for auto-align.
    private boolean wasTurningTurretManually = false;
    private boolean wasAutoAligning = false;

    // True = the turret tracks the hive all the time (ALWAYS-TRACK). The operator switches it with
    // the Right Stick Button, and a preset or A (STOP) switches it OFF.
    private boolean alwaysTrack = ALWAYS_TRACK_AT_START;
    private boolean wasTrackToggleButtonPressed = false;

    // The push the turret got this loop to follow the robot's spin, for the screen.
    private double turretFeedforward = 0.0;

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
        telemetry.addLine("Right Trigger (hold) = SHOOT (+ AUTO-ALIGN, even if ALWAYS-TRACK is off)");
        telemetry.addLine("X = gate CLOSED   B = gate OPENED   Y = gate LIFTED");
        telemetry.addLine("--- GAMEPAD 2 (OPERATOR) ---");
        telemetry.addLine("Start = flywheel speed DIAL / TABLE   Back = alliance BLUE / RED");
        telemetry.addLine("Right Stick Button = ALWAYS-TRACK on / off (turret follows the hive)");
        telemetry.addLine("D-pad Up / Down = flywheel RPM up / down");
        telemetry.addLine("D-pad Right / Left = ratio up / down");
        telemetry.addLine("Left Bumper = reverse BOTH wheels   Right Bumper = reverse flywheel only");
        telemetry.addLine("Left Stick X = turn turret");
        telemetry.addLine(String.format("Y = center (%.0f)  X = far left (%+.0f)  B = far right (%+.0f) degrees",
                TURRET_PRESET_CENTER_DEGREES, TURRET_PRESET_LEFT_DEGREES, TURRET_PRESET_RIGHT_DEGREES));
        telemetry.addLine("A = STOP turret");
        telemetry.addLine("Right Trigger = gate OPENED   Left Trigger = gate CLOSED");
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
     * Gamepad2 Start switches where the flywheel speed comes from (DIAL <-> TABLE), and
     * gamepad2 Back switches the alliance (BLUE <-> RED). One switch per press. This works
     * before and after START, so the operator can set both during INIT.
     */
    private void checkModeButtons() {
        boolean startPressed = gamepad2.start;
        boolean backPressed = gamepad2.back;

        if (isNewPress(startPressed, wasStartPressed)) {
            tableMode = !tableMode;
        }
        if (isNewPress(backPressed, wasBackPressed)) {
            if (visionSubsystem.getAlliance() == VisionSubsystem.Alliance.BLUE) {
                visionSubsystem.setAlliance(VisionSubsystem.Alliance.RED);
            } else {
                visionSubsystem.setAlliance(VisionSubsystem.Alliance.BLUE);
            }
        }

        wasStartPressed = startPressed;
        wasBackPressed = backPressed;
    }

    /*
     * Turns the turret toward the hive. Returns the words to show on the screen.
     * Call it every loop while ALWAYS-TRACK is ON or the driver's Right Trigger is held.
     */
    private String autoAlignTurret() {
        if (!visionSubsystem.hasTarget()) {
            // Nothing to aim at. If the turret is still running a manual stick power (the stick
            // was just let go) or is floating, hold where it is. If it already has a target
            // angle, it keeps working toward that angle. It does not search.
            if (turretSubsystem.getState() != TurretSubsystem.TurretState.POSITION) {
                turretSubsystem.holdCurrentAngle();
            }
            return "AUTO-ALIGN (NO TARGET - holding)";
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
        checkModeButtons();

        // The Driver Station clears its screen every loop, so we add the
        // button map again here. That keeps it on the screen until START.
        telemetry.addData("Status", "Ready to start!");
        telemetry.addData("Alliance (gamepad2 Back)", visionSubsystem.getAlliance());
        telemetry.addData("Flywheel speed source (gamepad2 Start)", getModeText());
        visionSubsystem.addTelemetry(telemetry);
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

        // 2. Read the gamepads and decide what to do.
        checkModeButtons();

        // ---- Gamepad 1 (driver) ----
        // On the controller, pushing a stick FORWARD gives a negative number, and pushing it RIGHT
        // gives a positive number. Our subsystem wants forward +, LEFT +, and COUNTERCLOCKWISE +,
        // so we flip the sign of all three. (Turning right is clockwise, which is negative.)
        double forward = applyDeadband(-gamepad1.left_stick_y);
        double strafe = applyDeadband(-gamepad1.left_stick_x);
        double turn = applyDeadband(-gamepad1.right_stick_x);

        boolean slowMode = gamepad1.left_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean triggerShotPressed = gamepad1.right_trigger > TRIGGER_PRESSED_THRESHOLD;

        // The driver's face buttons move the shooter gate by hand, for manual control and testing.
        boolean driverGateClosedPressed = gamepad1.x;
        boolean driverGateOpenedPressed = gamepad1.b;
        boolean driverGateLiftedPressed = gamepad1.y;

        // ---- Gamepad 2 (operator) ----
        boolean yPressed = gamepad2.y;
        boolean xPressed = gamepad2.x;
        boolean bPressed = gamepad2.b;
        boolean aPressed = gamepad2.a;
        boolean dpadUpPressed = gamepad2.dpad_up;
        boolean dpadDownPressed = gamepad2.dpad_down;
        boolean dpadLeftPressed = gamepad2.dpad_left;
        boolean dpadRightPressed = gamepad2.dpad_right;
        boolean gateShootTriggerPressed = gamepad2.right_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean gateLoadTriggerPressed = gamepad2.left_trigger > TRIGGER_PRESSED_THRESHOLD;
        boolean trackToggleButtonPressed = gamepad2.right_stick_button;

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
        driveSubsystem.drive(forward, strafe, turn);

        // Intake (driver bumpers). Check REVERSE first, so clearing a jam always works.
        String intakeControl;
        if (gamepad1.left_bumper) {
            intakeSubsystem.reverse();
            intakeControl = "Left Bumper (REVERSE)";
        } else if (gamepad1.right_bumper) {
            intakeSubsystem.intake();
            intakeControl = "Right Bumper (INTAKE)";
        } else {
            // No bumper held, so stop the intake.
            intakeControl = "None";
            intakeSubsystem.stop();
        }

        // Shooter wheels (operator). Reverse beats everything, so clearing a jam always works.
        String shooterControl;
        // Left Bumper reverses BOTH wheels. Right Bumper reverses only the flywheel.
        boolean reverseFlywheel = gamepad2.left_bumper || gamepad2.right_bumper;
        boolean reverseHoodWheel = gamepad2.left_bumper;
        if (reverseFlywheel || reverseHoodWheel) {
            // A wheel that is NOT being reversed gets 0 power.
            double reverseFlywheelPower = 0.0;
            double reverseHoodWheelPower = 0.0;
            if (reverseFlywheel) {
                reverseFlywheelPower = FLYWHEEL_POWER_REVERSE;
            }
            if (reverseHoodWheel) {
                reverseHoodWheelPower = HOOD_WHEEL_POWER_REVERSE;
            }
            shooterSubsystem.setPower(reverseFlywheelPower, reverseHoodWheelPower);
            shooterControl = "Bumper (REVERSE)";
        } else if (triggerShotPressed) {
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
        // The gate has two sets of buttons for now, and both work at the same time:
        //   operator (gamepad2): Left Trigger = CLOSED, Right Trigger = OPENED
        //   driver (gamepad1):   X = CLOSED, B = OPENED, Y = LIFTED
        // If more than one is pressed on the same loop, CLOSED wins, then OPENED, then LIFTED.
        boolean closedNewPress = isNewPress(gateLoadTriggerPressed, wasGateLoadTriggerPressed)
                || isNewPress(driverGateClosedPressed, wasDriverGateClosedPressed);
        boolean openedNewPress = isNewPress(gateShootTriggerPressed, wasGateShootTriggerPressed)
                || isNewPress(driverGateOpenedPressed, wasDriverGateOpenedPressed);
        boolean liftedNewPress = isNewPress(driverGateLiftedPressed, wasDriverGateLiftedPressed);
        if (closedNewPress) {
            shooterServoSubsystem.goToClosed();
        } else if (openedNewPress) {
            shooterServoSubsystem.goToOpened();
        } else if (liftedNewPress) {
            shooterServoSubsystem.goToLifted();
        }

        // Turret (operator, gamepad2). The same controls as TurretAlpha. The first one that is
        // true wins.

        // The turret turns at the same power every time. A full stick push is the whole power.
        double turretPower = TURRET_POWER_PERCENT / PERCENT_PER_ONE;

        // Stick RIGHT is a positive number on the controller. But turning RIGHT means going
        // clockwise, and clockwise is a NEGATIVE angle. So we flip the sign of the stick.
        double turretStick = -gamepad2.left_stick_x;
        boolean usingTurretStick = Math.abs(turretStick) > TURRET_STICK_DEADBAND;

        // Right Stick Button switches ALWAYS-TRACK ON <-> OFF. One switch per press.
        if (isNewPress(trackToggleButtonPressed, wasTrackToggleButtonPressed)) {
            alwaysTrack = !alwaysTrack;
        }

        // The operator picking a turret angle (Y, X, B) or STOP (A) means "stay where I put it",
        // so those switch ALWAYS-TRACK OFF. Do this BEFORE we decide below, so the press counts
        // on the very loop it happens. (The operator can press the Right Stick Button to turn
        // tracking back on.)
        if (yPressed || xPressed || bPressed || aPressed) {
            alwaysTrack = false;
        }

        // Auto-align runs ALL THE TIME while ALWAYS-TRACK is ON, and also while the driver holds
        // the Right Trigger (it aligns AND shoots, even if ALWAYS-TRACK is OFF).
        boolean alignWanted = alwaysTrack || triggerShotPressed;
        boolean autoAligningNow = false;

        // Cleared every loop. Only auto-align sets it, so nothing else ever gets a push.
        turretFeedforward = 0.0;

        String turretControl;
        if (usingTurretStick) {
            // The stick scales the power: a full push is the whole turret power,
            // a half push is half of it. The stick beats auto-align. When it is let go,
            // ALWAYS-TRACK (if it is ON) takes the turret back.
            turretSubsystem.setManualPower(turretStick * turretPower);
            turretControl = "Left Stick (MANUAL)";
        } else if (alignWanted && !aPressed) {
            // A (STOP) beats auto-align too: while A is held we skip this and the turret falls
            // through to the A branch below.
            turretControl = autoAlignTurret();
            autoAligningNow = true;
        } else if (yPressed) {
            turretSubsystem.setTargetAngle(TURRET_PRESET_CENTER_DEGREES);
            turretControl = "Y (preset CENTER)";
        } else if (xPressed) {
            turretSubsystem.setTargetAngle(TURRET_PRESET_LEFT_DEGREES);
            turretControl = "X (preset FAR LEFT)";
        } else if (bPressed) {
            turretSubsystem.setTargetAngle(TURRET_PRESET_RIGHT_DEGREES);
            turretControl = "B (preset FAR RIGHT)";
        } else if (aPressed) {
            turretSubsystem.stop();
            turretControl = "A (STOP)";
        } else if (wasTurningTurretManually || wasAutoAligning) {
            // The stick or the auto-align was just let go, so hold the turret where it stopped.
            turretSubsystem.holdCurrentAngle();
            turretControl = "Released (HOLD)";
        } else {
            // No new command. The turret keeps doing what it was last told to do.
            turretControl = "None (keeps last command)";
        }
        turretSubsystem.update();

        // Remember this loop's buttons for the next loop.
        wasDpadUpPressed = dpadUpPressed;
        wasDpadDownPressed = dpadDownPressed;
        wasTurningTurretManually = usingTurretStick;
        wasAutoAligning = autoAligningNow;
        wasTrackToggleButtonPressed = trackToggleButtonPressed;
        wasGateShootTriggerPressed = gateShootTriggerPressed;
        wasGateLoadTriggerPressed = gateLoadTriggerPressed;
        wasDriverGateClosedPressed = driverGateClosedPressed;
        wasDriverGateOpenedPressed = driverGateOpenedPressed;
        wasDriverGateLiftedPressed = driverGateLiftedPressed;
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
        telemetry.addData("Alliance (gamepad2 Back)", visionSubsystem.getAlliance());

        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive Control", driveControl);
        telemetry.addData("Drive inputs", "forward %.2f   strafe %.2f   turn %.2f",
                forward, strafe, turn);

        // The dials: what the operator has asked for. (They only take effect while the driver
        // holds the Right Trigger.)
        telemetry.addLine("--- SHOOTER DIALS (RPM) ---");
        telemetry.addData("Flywheel speed source (gamepad2 Start)", getModeText());
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
        } else if (shooterState == ShooterSubsystem.ShooterState.OPEN_LOOP) {
            readyText = "NO - reversing (clearing a jam)";
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
        telemetry.addData("ALWAYS-TRACK (gamepad2 Right Stick Button)",
                alwaysTrack ? "ON - the turret follows the hive" : "OFF - the turret stays where you put it");
        telemetry.addData("Turret power", "%d%%", TURRET_POWER_PERCENT);
        telemetry.addData("Turret follows spin", "%+.2f power   (gain %.2f)",
                turretFeedforward, TURRET_FEEDFORWARD_GAIN);

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
    }
}
