package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.PowerMonitorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Firing Monitor
 * Watches the firing sequence so we can find out WHY the gate does not always open on time
 * (issue #122: "Firing sequence is requiring too much manual override").
 *
 * WHAT IT IS (and is not)
 *   A plain class with NO hardware and NO gamepad. It does not move anything and it never changes
 *   what the robot does. Each loop the TeleOp hands it a few yes/no answers, and it only LOOKS at
 *   the subsystems (through their get...() methods). Like BiobuzzLog and FiringSequence, its name
 *   does not end in "Subsystem" because it is not a mechanism.
 *
 * IT DOES TWO JOBS
 *   1. THE VOLLEY LOG (always on, no laptop needed). Every volley, it writes two lines to the Robot
 *      Controller log (see BiobuzzLog for how to read it, search for BIOBUZZ):
 *        "VOLLEY WAIT: ..."  how long the volley waited for the green light, and WHAT it waited on:
 *                            the flywheel only, the turret only, or both. It also says how fast the
 *                            flywheel was going when the trigger was pulled (a flywheel that starts
 *                            from 0 RPM needs seconds to spin up, PRE-REV avoids that), whether
 *                            PRE-REV was on, TABLE or DIAL, and the battery.
 *        "VOLLEY FEED: ..."  how the feed went: how long, how many flywheel dips (about one per ball),
 *                            how long the intake was paused, and the lowest flywheel RPM and battery
 *                            voltage during the feed.
 *      A wait that ends in "ABORTED" means the trigger was let go before the volley was ready.
 *   2. LIVE GRAPHS IN PANELS (practice). It hands numbers to PanelsGraphs every loop, so a laptop
 *      on the robot's wifi can show them as graphs. PanelsGraphs has the master switch that turns
 *      Panels on or off for every OpMode (issue #138), and it owns the one Panels connection.
 *      Panels sends to the browser about 13 times a second (its own limit), so very short blips can
 *      be missed. Every graph number here is a plain number with NO units in it (the units are in
 *      the name). Yes/no answers are sent as 1 (yes) and 0 (no) so they draw as steps.
 *
 * WHAT TO LOOK FOR IN THE GRAPHS
 *   - "Flywheel RPM" against "Flywheel target": does it sit BELOW the target while it is running
 *     (a steady droop), wobble around it (a noisy reading), or just take a long time to climb from
 *     0 (spin-up)? Those three have three different fixes.
 *   - "Flywheel error (RPM)" against the two "At-speed band" lines: the flywheel counts as at speed
 *     only while the error is between them (and has stayed there for AT_SPEED_HOLD_SECONDS).
 *   - "Flywheel RPM (raw)" against "Flywheel RPM": the raw reading is jumpier. If the smoothed one
 *     lags far behind the raw one, the smoothing is hiding what is really happening.
 *   - "Flywheel amps": a flywheel below its target while drawing a lot of current has no power left
 *     to catch up (weak battery, belt rubbing). Below its target while drawing little is tuning.
 *   - The 1/0 steps ("At speed", "Turret ready", "Ready to start", "Gate", "Sequence step"): when
 *     the operator reaches for Y or B ("Y lift held", "Forced shot held"), which of the two
 *     halves was still 0?
 *
 * HOW TO USE IT (once per loop, AFTER the firing sequence, the gate and the intake have been
 * updated, so every answer is fresh. It must come AFTER panelsGraphs.beginLoop() and BEFORE
 * panelsGraphs.send(), because the graph numbers go out through PanelsGraphs):
 *   firingMonitor.update(fireHeld, forcedShotHeld, liftHeld, turretReady, readyToStart,
 *                        tableMode, preRevOn, waitingText);
 *
 * WHY IT DOES NOT TALK TO PANELS ITSELF: PanelsGraphs is the ONLY file that uses the Panels
 * library. Panels throws away a second update() in the same loop, so two classes must not each
 * send their own. If Panels ever causes trouble, turn the master switch off in PanelsGraphs
 * and the volley log (job 1) keeps working.
 */
public class FiringMonitor {

    // ------------------------------------------------------------------
    // Tunable numbers
    // ------------------------------------------------------------------
    // true  = also graph the flywheel's current (amps). This ASKS THE HUB while the wheels are
    //         running, so it makes the loop a little slower (see "Loop (ms)" in the graphs to
    //         measure how much). It only happens when Panels is on (the master switch in
    //         PanelsGraphs), and only on the loops Panels will really send (about 13 a second).
    // false = leave the current out.
    private static final boolean GRAPH_FLYWHEEL_CURRENT = true;

    // A loop longer than this is not counted in the "waited on" times. The first loop after a long
    // pause (or a hiccup) would otherwise add a huge wait that never happened.
    private static final double MAX_COUNTED_LOOP_SECONDS = 0.25;

    // ------------------------------------------------------------------
    // What we watch (the OpMode gives them to us. We never change them.)
    // ------------------------------------------------------------------
    private final ShooterSubsystem shooterSubsystem;
    private final ShooterServoSubsystem shooterServoSubsystem;
    private final FiringSequence firingSequence;
    private final VisionSubsystem visionSubsystem;
    private final PowerMonitorSubsystem powerMonitorSubsystem;

    // How many degrees off the camera's tx may be and still count as aimed. It is the TeleOp's
    // number (AIMED_TOLERANCE_DEGREES), handed in once so we draw the same limit.
    private final double aimToleranceDegrees;

    // Where the graph numbers go. (It owns the one Panels connection and the master switch.)
    private final PanelsGraphs panelsGraphs;

    // ------------------------------------------------------------------
    // The answers for THIS loop. update() fills them in first, so the helper methods can read them.
    // ------------------------------------------------------------------
    private FiringSequence.SequenceState step = FiringSequence.SequenceState.IDLE;
    private boolean flywheelAtSpeed = false;
    private boolean turretReady = false;
    private boolean readyToStart = false;
    private boolean liftHeld = false;
    private boolean tableMode = false;
    private boolean preRevOn = false;
    private double batteryVolts = 0.0;
    private String waitingText = "";
    private double loopSeconds = 0.0;

    // ------------------------------------------------------------------
    // The volley log
    // ------------------------------------------------------------------
    // Times the gap between two update() calls.
    private final ElapsedTime loopTimer = new ElapsedTime();

    // The step the sequence was on LAST loop, so we can see when it changes.
    private FiringSequence.SequenceState previousStep = FiringSequence.SequenceState.IDLE;

    // While the volley WAITS for the green light, which half was holding it up? Each loop's time
    // is added to ONE of these three.
    private double waitFlywheelOnlySeconds = 0.0;
    private double waitTurretOnlySeconds = 0.0;
    private double waitBothSeconds = 0.0;

    // Was the lift button (Y) held while the volley waited? (That is the operator's override.)
    private boolean liftHeldDuringWait = false;

    // What things looked like when the trigger was pulled.
    private double flywheelRpmAtPull = 0.0;
    private boolean preRevAtPull = false;
    private boolean tableModeAtPull = false;

    // The last reason the volley gave for not being ready ("flywheel not at speed", ...).
    private String lastBlockingReason = "";

    // The flywheel's RPM and target on the LAST loop of the wait. We remember them while the wait
    // runs, because by the time we write the log line the wheels may already be stopped (when the
    // trigger is let go, the TeleOp turns them off in that same loop, and their target reads 0).
    private double waitLastFlywheelRpm = 0.0;
    private double waitLastTargetRpm = 0.0;

    // The feed. (The target is remembered while the feed runs, for the same reason as above.)
    private final ElapsedTime feedTimer = new ElapsedTime();
    private double feedLowestFlywheelRpm = 0.0;
    private double feedLowestBatteryVolts = 0.0;
    private double feedTargetRpm = 0.0;

    // The last two log lines, kept so Panels can show them too.
    private String lastWaitText = "none yet";
    private String lastFeedText = "none yet";

    // ==================================================================
    // SETUP
    // ==================================================================

    /*
     * panelsGraphs is the OpMode's one PanelsGraphs (it decides whether the graphs are on, see its
     * master switch). The volley log is written either way.
     */
    public FiringMonitor(ShooterSubsystem shooterSubsystem,
                         ShooterServoSubsystem shooterServoSubsystem,
                         FiringSequence firingSequence,
                         VisionSubsystem visionSubsystem,
                         PowerMonitorSubsystem powerMonitorSubsystem,
                         double aimToleranceDegrees,
                         PanelsGraphs panelsGraphs) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterServoSubsystem = shooterServoSubsystem;
        this.firingSequence = firingSequence;
        this.visionSubsystem = visionSubsystem;
        this.powerMonitorSubsystem = powerMonitorSubsystem;
        this.aimToleranceDegrees = aimToleranceDegrees;
        this.panelsGraphs = panelsGraphs;
    }

    // ==================================================================
    // ACTION: the OpMode calls this once per loop
    // ==================================================================

    /*
     * Call this ONCE per loop, after the firing sequence, the gate and the intake were updated.
     *   fireHeld        - is the fire trigger held right now?
     *   forcedShotHeld  - is the FORCED SHOT button (B) held right now?
     *   liftHeld        - is the gate LIFT button (Y) held right now?
     *   turretReady     - the TeleOp's "turret aimed" answer (the same one the sequence got)
     *   readyToStart    - the TeleOp's "flywheel at speed AND turret ready" answer
     *   tableMode       - true = the flywheel speed comes from the shot table, false = the dial
     *   preRevOn        - is PRE-REV on?
     *   waitingText     - the TeleOp's words for what the volley is waiting for. Only needed while
     *                     the sequence is in WAIT_FOR_READY. Pass "" at any other time.
     * (The "Loop (ms)" graph is added by PanelsGraphs itself, so it is not passed in any more.)
     */
    public void update(boolean fireHeld, boolean forcedShotHeld, boolean liftHeld,
                       boolean turretReady, boolean readyToStart, boolean tableMode,
                       boolean preRevOn, String waitingText) {
        // Time since the last call. Cap it so one hiccup cannot add a wait that never happened.
        loopSeconds = Math.min(loopTimer.seconds(), MAX_COUNTED_LOOP_SECONDS);
        loopTimer.reset();

        // Write down this loop's answers where the helpers can read them.
        step = firingSequence.getState();
        flywheelAtSpeed = shooterSubsystem.isFlywheelAtSpeed();
        this.turretReady = turretReady;
        this.readyToStart = readyToStart;
        this.liftHeld = liftHeld;
        this.tableMode = tableMode;
        this.preRevOn = preRevOn;
        this.waitingText = waitingText;
        batteryVolts = powerMonitorSubsystem.getLowestVoltageNow();

        trackWait();
        trackFeed();

        // Only build the graph numbers on a loop that Panels will really send (see PanelsGraphs).
        if (panelsGraphs.isFrameOpen()) {
            addGraphNumbers(fireHeld, forcedShotHeld);
        }

        previousStep = step;
    }

    // ==================================================================
    // THE VOLLEY LOG
    // ==================================================================

    // The WAIT step: the trigger is held, the gate is CLOSED, and we are waiting for the green light.
    private void trackWait() {
        if (step == FiringSequence.SequenceState.WAIT_FOR_READY) {
            if (previousStep != FiringSequence.SequenceState.WAIT_FOR_READY) {
                startWait();
            }
            addWaitTime();
        } else if (previousStep == FiringSequence.SequenceState.WAIT_FOR_READY) {
            // We just LEFT the wait. The new step says how it ended.
            finishWait();
        }
    }

    // The trigger was just pulled. Start the "waited on" clocks over and note how things looked.
    private void startWait() {
        waitFlywheelOnlySeconds = 0.0;
        waitTurretOnlySeconds = 0.0;
        waitBothSeconds = 0.0;
        liftHeldDuringWait = false;
        lastBlockingReason = "";
        flywheelRpmAtPull = shooterSubsystem.getFlywheelRpm();
        preRevAtPull = preRevOn;
        tableModeAtPull = tableMode;
    }

    // One loop of waiting: add its time to the half (or both) that was NOT ready.
    private void addWaitTime() {
        if (!flywheelAtSpeed && !turretReady) {
            waitBothSeconds = waitBothSeconds + loopSeconds;
        } else if (!flywheelAtSpeed) {
            waitFlywheelOnlySeconds = waitFlywheelOnlySeconds + loopSeconds;
        } else if (!turretReady) {
            waitTurretOnlySeconds = waitTurretOnlySeconds + loopSeconds;
        }
        if (liftHeld) {
            liftHeldDuringWait = true;
        }
        if (!readyToStart && !waitingText.isEmpty()) {
            lastBlockingReason = waitingText.trim();
        }
        waitLastFlywheelRpm = shooterSubsystem.getFlywheelRpm();
        waitLastTargetRpm = shooterSubsystem.getTargetFlywheelRpm();
    }

    // The wait is over. step is the NEW step: FEED (it got ready), FORCED (B was pressed before it
    // got ready), or IDLE (the trigger was let go while it still waited).
    private void finishWait() {
        String how;
        if (step == FiringSequence.SequenceState.FEED) {
            how = "READY";
        } else if (step == FiringSequence.SequenceState.FORCED) {
            how = "FORCED SHOT (B) pressed before it was ready";
        } else {
            how = "ABORTED (trigger let go while waiting)";
        }

        double totalSeconds = waitFlywheelOnlySeconds + waitTurretOnlySeconds + waitBothSeconds;
        String source = "DIAL";
        if (tableModeAtPull) {
            source = "TABLE";
        }
        String preRev = "off";
        if (preRevAtPull) {
            preRev = "ON";
        }
        String liftNote = "";
        if (liftHeldDuringWait) {
            liftNote = " Y (lift) was held while waiting.";
        }
        String reasonNote = "";
        if (!lastBlockingReason.isEmpty()) {
            reasonNote = " Last reason: " + lastBlockingReason + ".";
        }

        lastWaitText = String.format(
                "%s after %.2f s. Waited on: flywheel only %.2f s, turret only %.2f s, both %.2f s.%s%s"
                        + " At the pull: flywheel %.0f RPM, PRE-REV %s, %s. End of the wait: flywheel %.0f of %.0f RPM. Battery %.2f V.",
                how, totalSeconds, waitFlywheelOnlySeconds, waitTurretOnlySeconds, waitBothSeconds,
                liftNote, reasonNote,
                flywheelRpmAtPull, preRev, source,
                waitLastFlywheelRpm, waitLastTargetRpm,
                batteryVolts);
        BiobuzzLog.info("VOLLEY WAIT: %s", lastWaitText);
    }

    // The FEED step: the gate is open and balls are going through.
    private void trackFeed() {
        if (step == FiringSequence.SequenceState.FEED) {
            if (previousStep != FiringSequence.SequenceState.FEED) {
                feedTimer.reset();
                feedLowestFlywheelRpm = shooterSubsystem.getFlywheelRpm();
                feedLowestBatteryVolts = batteryVolts;
            }
            if (shooterSubsystem.getFlywheelRpm() < feedLowestFlywheelRpm) {
                feedLowestFlywheelRpm = shooterSubsystem.getFlywheelRpm();
            }
            feedTargetRpm = shooterSubsystem.getTargetFlywheelRpm();
            // 0.0 means "the hubs gave no reading", so it must not count as the lowest.
            if (batteryVolts > 0.0 && (feedLowestBatteryVolts <= 0.0 || batteryVolts < feedLowestBatteryVolts)) {
                feedLowestBatteryVolts = batteryVolts;
            }
        } else if (previousStep == FiringSequence.SequenceState.FEED) {
            finishFeed();
        }
    }

    private void finishFeed() {
        lastFeedText = String.format(
                "Fed for %.1f s. Flywheel dips %d (about 1 per ball). Intake paused %.1f s."
                        + " Flywheel low %.0f RPM (target %.0f). Battery low %.2f V.",
                feedTimer.seconds(), firingSequence.getFlywheelDipCount(), firingSequence.getPausedSeconds(),
                feedLowestFlywheelRpm, feedTargetRpm, feedLowestBatteryVolts);
        BiobuzzLog.info("VOLLEY FEED: %s", lastFeedText);
    }

    // ==================================================================
    // THE GRAPH NUMBERS (for Panels, through PanelsGraphs)
    // ==================================================================

    // PanelsGraphs sends each number to Panels as "name: value", and Panels graphs the ones that are
    // plain numbers. So: NO units inside the value (put them in the name), 1 and 0 for yes and no,
    // and the SAME names every loop (a graph follows a name over time). The shared sets (the
    // flywheel, the camera, the gate) live in PanelsGraphs, so every OpMode uses the same names.
    // This only runs on a loop that PanelsGraphs will really send. The OpMode itself calls
    // panelsGraphs.send() once at the end of loop(), so we never call it here.
    private void addGraphNumbers(boolean fireHeld, boolean forcedShotHeld) {
        // --- THE FLYWHEEL (and the hood wheel) ---
        panelsGraphs.addShooter(shooterSubsystem, GRAPH_FLYWHEEL_CURRENT);

        // --- THE BATTERY ---
        panelsGraphs.add("Battery (V)", batteryVolts);

        // --- THE GREEN LIGHT: both halves, and the sequence (1 = yes, 0 = no) ---
        panelsGraphs.addFlag("At speed (1=yes)", flywheelAtSpeed);
        panelsGraphs.addFlag("Turret ready (1=yes)", turretReady);
        panelsGraphs.addFlag("Ready to start (1=yes)", readyToStart);
        panelsGraphs.add("Sequence step (0 idle, 1 wait, 2 feed, 3 done, 4 forced)", getStepNumber());
        panelsGraphs.addGate(shooterServoSubsystem);
        panelsGraphs.addFlag("Intake wanted by sequence (1=yes)",
                firingSequence.ownsIntake() && firingSequence.wantsIntake());

        // --- THE CAMERA ---
        panelsGraphs.addVision(visionSubsystem);
        panelsGraphs.add("Aim limit +", aimToleranceDegrees);
        panelsGraphs.add("Aim limit -", -aimToleranceDegrees);

        // --- THE PEOPLE (what the driver and operator are doing) ---
        panelsGraphs.addFlag("Trigger held (1=yes)", fireHeld);
        panelsGraphs.addFlag("Y lift held (1=yes)", liftHeld);
        panelsGraphs.addFlag("Forced shot held (1=yes)", forcedShotHeld);
        panelsGraphs.addFlag("PRE-REV on (1=yes)", preRevOn);
        panelsGraphs.addFlag("TABLE mode (1=table, 0=dial)", tableMode);

        // --- WORDS (they show on Panels' Telemetry widget, they are not graphed) ---
        panelsGraphs.addText("Step", step.toString());
        if (!waitingText.isEmpty()) {
            panelsGraphs.addText("Waiting for", waitingText.trim());
        }
        panelsGraphs.addText("Last wait", lastWaitText);
        panelsGraphs.addText("Last feed", lastFeedText);
    }

    // The step as a number, so it can be drawn as a graph.
    private int getStepNumber() {
        if (step == FiringSequence.SequenceState.WAIT_FOR_READY) {
            return 1;
        }
        if (step == FiringSequence.SequenceState.FEED) {
            return 2;
        }
        if (step == FiringSequence.SequenceState.DONE) {
            return 3;
        }
        if (step == FiringSequence.SequenceState.FORCED) {
            return 4;
        }
        return 0;
    }
}
