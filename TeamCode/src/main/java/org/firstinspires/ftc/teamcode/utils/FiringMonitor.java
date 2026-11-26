package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 *   2. LIVE GRAPHS IN PANELS (only when the TeleOp turns it on, practice only). It sends numbers to
 *      Panels every loop, so a laptop on the robot's wifi can show them as graphs. Panels sends them
 *      to the browser about 13 times a second (its own limit), so very short blips can be missed.
 *      Panels draws a number as a graph when it is sent as "name: number", so every graph number
 *      here is a plain number with NO units in it (the units are in the name). Yes/no answers are
 *      sent as 1 (yes) and 0 (no) so they draw as steps.
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
 * updated, so every answer is fresh):
 *   firingMonitor.update(fireHeld, forcedShotHeld, liftHeld, turretReady, readyToStart,
 *                        tableMode, preRevOn, loopMs, waitingText);
 *
 * WHY IT IS ITS OWN CLASS: it is the ONLY file that uses the Panels library. If Panels ever causes
 * trouble, delete this file's Panels lines (or turn the switch off in BiobuzzTeleOp) and nothing
 * else in the robot code changes.
 */
public class FiringMonitor {

    // ------------------------------------------------------------------
    // Tunable numbers
    // ------------------------------------------------------------------
    // true  = also graph the flywheel's current (amps). This ASKS THE HUB every loop while the
    //         wheels are running, so it makes the loop a little slower (see "Loop (ms)" in the
    //         graphs to measure how much). It only happens when Panels is on, which is practice only.
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

    // The Panels telemetry. null means "not sending to Panels" (it was switched off, or Panels
    // could not be started).
    private TelemetryManager panels = null;

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
     * sendToPanels = true turns the live graphs on (practice only). false keeps only the volley log.
     * If Panels cannot be started for any reason, we say so in the log and carry on without it:
     * the robot must never stop because of a graph.
     */
    public FiringMonitor(ShooterSubsystem shooterSubsystem,
                         ShooterServoSubsystem shooterServoSubsystem,
                         FiringSequence firingSequence,
                         VisionSubsystem visionSubsystem,
                         PowerMonitorSubsystem powerMonitorSubsystem,
                         double aimToleranceDegrees,
                         boolean sendToPanels) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterServoSubsystem = shooterServoSubsystem;
        this.firingSequence = firingSequence;
        this.visionSubsystem = visionSubsystem;
        this.powerMonitorSubsystem = powerMonitorSubsystem;
        this.aimToleranceDegrees = aimToleranceDegrees;

        if (sendToPanels) {
            try {
                // PanelsTelemetry is a Kotlin "object", so in Java we reach it through INSTANCE.
                panels = PanelsTelemetry.INSTANCE.getTelemetry();
            } catch (Exception e) {
                panels = null;
                BiobuzzLog.warning("FiringMonitor: Panels could not be started (%s). Graphs are OFF.", e.toString());
            }
        }
    }

    // true = the numbers are going to Panels.
    public boolean isSendingToPanels() {
        return panels != null;
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
     *   loopMs          - how long the last loop took, in milliseconds (the TeleOp measures it)
     *   waitingText     - the TeleOp's words for what the volley is waiting for. Only needed while
     *                     the sequence is in WAIT_FOR_READY. Pass "" at any other time.
     */
    public void update(boolean fireHeld, boolean forcedShotHeld, boolean liftHeld,
                       boolean turretReady, boolean readyToStart, boolean tableMode,
                       boolean preRevOn, double loopMs, String waitingText) {
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

        if (panels != null) {
            sendToPanels(fireHeld, forcedShotHeld, loopMs);
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
    // PANELS (practice only)
    // ==================================================================

    // Panels shows each addData(name, value) as a line "name: value", and graphs the ones that are
    // plain numbers. So: NO units inside the value (put them in the name), 1 and 0 for yes and no,
    // and the SAME names every loop (a graph follows a name over time).
    private void sendToPanels(boolean fireHeld, boolean forcedShotHeld, double loopMs) {
        double flywheelRpm = shooterSubsystem.getFlywheelRpm();
        double flywheelTarget = shooterSubsystem.getTargetFlywheelRpm();
        double tolerance = shooterSubsystem.getAtSpeedToleranceRpm();

        // --- THE FLYWHEEL ---
        panels.addData("Flywheel RPM", round1(flywheelRpm));
        panels.addData("Flywheel RPM (raw)", round1(shooterSubsystem.getFlywheelRawRpm()));
        panels.addData("Flywheel target", round1(flywheelTarget));
        // Negative = too slow.
        panels.addData("Flywheel error (RPM)", round1(flywheelRpm - flywheelTarget));
        panels.addData("At-speed band +", round1(tolerance));
        panels.addData("At-speed band -", round1(-tolerance));
        // The current is only asked for while the wheels run, because asking costs loop time.
        if (GRAPH_FLYWHEEL_CURRENT && shooterSubsystem.getState() != ShooterSubsystem.ShooterState.IDLE) {
            panels.addData("Flywheel amps", round1(shooterSubsystem.getFlywheelCurrentAmps()));
        }
        // The hood wheel runs about twice as fast, so it gets its own names (and its own graph).
        panels.addData("Hood wheel RPM", round1(shooterSubsystem.getHoodWheelRpm()));
        panels.addData("Hood wheel target", round1(shooterSubsystem.getTargetHoodWheelRpm()));

        // --- THE BATTERY AND THE LOOP ---
        panels.addData("Battery (V)", round2(batteryVolts));
        panels.addData("Loop (ms)", round1(loopMs));

        // --- THE GREEN LIGHT: both halves, and the sequence (1 = yes, 0 = no) ---
        panels.addData("At speed (1=yes)", flag(flywheelAtSpeed));
        panels.addData("Turret ready (1=yes)", flag(turretReady));
        panels.addData("Ready to start (1=yes)", flag(readyToStart));
        panels.addData("Sequence step (0 idle, 1 wait, 2 feed, 3 done, 4 forced)", getStepNumber());
        panels.addData("Gate (0 closed, 1 opened, 2 lifted)", getGateNumber());
        panels.addData("Intake wanted by sequence (1=yes)", flag(firingSequence.ownsIntake() && firingSequence.wantsIntake()));

        // --- THE CAMERA ---
        boolean hasTarget = visionSubsystem.hasTarget();
        panels.addData("Camera sees hive (1=yes)", flag(hasTarget));
        // tx only means something while the camera sees the hive, so no tx number without it.
        if (hasTarget) {
            panels.addData("Camera tx (deg)", round1(visionSubsystem.getTxDegrees()));
        }
        panels.addData("Aim limit +", round1(aimToleranceDegrees));
        panels.addData("Aim limit -", round1(-aimToleranceDegrees));

        // --- THE PEOPLE (what the driver and operator are doing) ---
        panels.addData("Trigger held (1=yes)", flag(fireHeld));
        panels.addData("Y lift held (1=yes)", flag(liftHeld));
        panels.addData("Forced shot held (1=yes)", flag(forcedShotHeld));
        panels.addData("PRE-REV on (1=yes)", flag(preRevOn));
        panels.addData("TABLE mode (1=table, 0=dial)", flag(tableMode));

        // --- WORDS ---
        panels.addData("Step", step.toString());
        if (!waitingText.isEmpty()) {
            panels.addData("Waiting for", waitingText.trim());
        }
        panels.addData("Last wait", lastWaitText);
        panels.addData("Last feed", lastFeedText);

        // Send it. (This is Panels' own screen, so it does NOT touch the Driver Station's telemetry.
        // Panels keeps its own pace and may skip a call if the last send was very recent.)
        panels.update();
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

    // The gate as a number, so it can be drawn as a graph. (What we COMMANDED: a servo cannot say
    // where it really is.)
    private int getGateNumber() {
        ShooterServoSubsystem.GateState gate = shooterServoSubsystem.getState();
        if (gate == ShooterServoSubsystem.GateState.OPENED) {
            return 1;
        }
        if (gate == ShooterServoSubsystem.GateState.LIFTED) {
            return 2;
        }
        return 0;
    }

    private int flag(boolean value) {
        if (value) {
            return 1;
        }
        return 0;
    }

    // Round to one decimal place, so Panels does not send 1299.8765432.
    private double round1(double value) {
        return Math.round(value * 10.0) / 10.0;
    }

    private double round2(double value) {
        return Math.round(value * 100.0) / 100.0;
    }
}
