package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Firing Sequence
 * The steps of one "volley": wait until we are ready, open the gate, and keep feeding balls until the
 * fire button is let go. It is the same sequence the Auto (ShootAndParkAuto) works out for itself, so
 * the TeleOp and the Auto can share one copy.
 *
 * WHAT IT IS (and is not)
 *   This is a plain class with NO hardware and NO gamepad. It is not a mechanism, so its name does
 *   not end in "Subsystem" (like ShotTable and HiveBearingMemory). It does not move anything. Each
 *   loop the OpMode tells it a few yes/no answers, and it answers with a gate command and whether
 *   the intake should run. The OpMode then gives those orders to ShooterServoSubsystem and
 *   IntakeSubsystem, which still own their hardware.
 *
 * HOW TO USE IT (once per loop, AFTER shooterSubsystem.update(), because the answers must be fresh)
 *   firingSequence.update(fireHeld, readyToStart, flywheelAtSpeed, turretAimed);
 *     fireHeld       - is the fire button held right now? (The OpMode reads the gamepad.)
 *     readyToStart   - may the first ball go? The OpMode decides: the flywheel at speed, the turret
 *                      on target, and (when the speed comes from the shot table) the camera sees the
 *                      hive right now.
 *     flywheelAtSpeed- is the flywheel at speed right now? (ShooterSubsystem.isFlywheelAtSpeed().)
 *                      It paces the feed, and counts the dips.
 *     turretAimed    - is the turret aimed right now? (The same turret answer the OpMode used for
 *                      readyToStart.) While feeding, the intake PAUSES whenever this is false, so
 *                      balls are not shot while the turret is off the hive.
 *   Then ask for the orders:
 *     getGateCommand()  - NONE means "I am not in charge of the gate, use your manual buttons".
 *                         CLOSE / OPEN means "send this to ShooterServoSubsystem".
 *     ownsIntake()      - true means "I am in charge of the intake". Then wantsIntake() says run it
 *                         or stop it. When ownsIntake() is false, the OpMode's own intake buttons
 *                         work as usual.
 *
 * THE STEPS (a volley starts when fireHeld goes true, and ends when it goes false)
 *   1. WAIT_FOR_READY : gate CLOSED. Waits for readyToStart. (The intake is NOT mine yet, so the
 *                       driver can still pick up balls. The gate is closed, so they are held.)
 *   2. FEED           : gate OPENED, and it STAYS open until the button is released. The intake runs
 *                       only while the flywheel is at speed (INTAKE_WAITS_FOR_SPEED) AND the turret
 *                       is aimed (INTAKE_WAITS_FOR_AIM), so balls go one at a time and only while we
 *                       are on target. There is no timer: shoot 3 or 4 balls (or more, if more
 *                       come in), then let go.
 *   ABORT / END: releasing the button at any step closes the gate once and gives the intake back. Any
 *   balls not yet shot stay in the chute.
 *
 * AN OPTIONAL FEED LIMIT (for the Auto, which has no button to let go of)
 *   setMaxFeedSeconds(seconds) stops the feed after that many seconds: the gate CLOSES, the step
 *   becomes DONE, and it stays DONE until the button is released (so it cannot fire again by
 *   itself). The default is NO limit (the TeleOp never sets one): the operator decides when to stop.
 *
 * THE LAST BALL (issue #63): the gate has a LIFTED position (OPENED is between LIFTED and CLOSED)
 * that pushes the very last ball up into the flywheel, when there are no balls behind it to push.
 * The sequence does not lift by itself (its timer was off and wedged balls, so it was removed).
 * Because the gate now stays OPENED while the button is held, the operator can lift by hand at the
 * right moment, and the OpMode returns the gate to OPENED when the lift button is let go. That is
 * the OpMode's job, not this class's.
 *
 * WHY THE GATE STAYS OPEN DURING THE FEED
 *   Opening and closing it for every ball would add the servo's travel time (about half a second
 *   each way, a guess) to every ball. The flywheel dip after each ball (when there is one: the balls
 *   are light) does the pacing instead.
 *
 * NO BALL SENSOR: we cannot count balls. The dip counter (about one dip per ball shot, if the balls
 * are heavy enough to show one) is only a check.
 */
public class FiringSequence {

    // ------------------------------------------------------------------
    // Tunable numbers
    // ------------------------------------------------------------------
    // The feed limit that means "no limit": feed until the button is let go.
    public static final double NO_FEED_LIMIT = 0.0;

    // true  = during the feed, the intake runs only while the flywheel is at speed (one ball at a
    //         time).
    // false = the intake ignores the flywheel. Use this to check that the intake moves the balls
    //         through the chute at all.
    private static final boolean INTAKE_WAITS_FOR_SPEED = true;

    // true  = during the feed, the intake pauses whenever the turret is not aimed (the camera says
    //         it is off the hive by more than the tolerance, or the camera lost the hive in TABLE
    //         mode), and starts again when it is aimed again. The OpMode decides what "aimed" means.
    // false = the intake ignores the turret.
    private static final boolean INTAKE_WAITS_FOR_AIM = true;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    // STATE = which step the volley is on.
    public enum SequenceState {
        IDLE,            // No volley. The fire button is not held.
        WAIT_FOR_READY,  // Button held. Gate CLOSED. Waiting for readyToStart.
        FEED,            // Gate OPENED. Feeding balls, until the button is released.
        DONE             // Only with a feed limit: the limit ran out. Gate CLOSED. Waiting for release.
    }

    // What the OpMode should tell the gate this loop.
    public enum GateCommand {
        NONE,   // Not my job this loop. Use the manual buttons.
        CLOSE,
        OPEN
    }

    private SequenceState state = SequenceState.IDLE;

    // The feed limit in seconds. NO_FEED_LIMIT (0) means feed until the button is released.
    private double maxFeedSeconds = NO_FEED_LIMIT;

    // The orders for THIS loop. update() works them out again every loop.
    private GateCommand gateCommand = GateCommand.NONE;
    private boolean ownsIntake = false;
    private boolean intakeWanted = false;
    private String intakePauseReason = "";

    // Times the current step, and the whole volley.
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime volleyTimer = new ElapsedTime();

    // The flywheel dip count. A dip is a loop where the flywheel was at speed last loop and is not
    // now. Each ball that goes through can slow the flywheel, so this is about one per ball.
    private boolean flywheelWasAtSpeed = false;
    private int flywheelDipCount = 0;

    // How long the intake was paused during the feed, in seconds (a check on how often the flywheel
    // or the turret held the balls back).
    private double pausedSeconds = 0.0;
    private double lastLoopSeconds = 0.0;

    // The log of the last volley. They stay until the next volley starts. -1 means "has not
    // happened (yet)".
    private double secondsToReady = -1.0;
    private double volleySeconds = -1.0;
    private String lastResult = "none yet";

    // ==================================================================
    // SETUP
    // ==================================================================

    /*
     * Limit the feed to this many seconds (the Auto uses this). Pass NO_FEED_LIMIT (the default) to
     * feed until the button is released (the TeleOp).
     */
    public void setMaxFeedSeconds(double seconds) {
        maxFeedSeconds = seconds;
    }

    // ==================================================================
    // ACTION: the OpMode calls this once per loop
    // ==================================================================

    /*
     * Call this ONCE per loop. See the top of this file for what the answers mean. After it, ask
     * getGateCommand(), ownsIntake() and wantsIntake().
     */
    public void update(boolean fireHeld, boolean readyToStart, boolean flywheelAtSpeed, boolean turretAimed) {
        // The orders are only for this loop. Start from "not my job".
        gateCommand = GateCommand.NONE;
        ownsIntake = false;
        intakeWanted = false;
        intakePauseReason = "";

        // How long since the last loop, to add up the paused time.
        double nowSeconds = volleyTimer.seconds();
        double loopSeconds = nowSeconds - lastLoopSeconds;
        lastLoopSeconds = nowSeconds;

        // The button is not held: no volley. If one was running, it is over (ended or aborted).
        if (!fireHeld) {
            if (state != SequenceState.IDLE) {
                if (state == SequenceState.FEED) {
                    // The normal way to end a volley: the operator let go.
                    lastResult = "ended (let go while feeding)";
                    volleySeconds = volleyTimer.seconds();
                } else if (state == SequenceState.DONE) {
                    lastResult = "finished (feed limit)";
                } else {
                    lastResult = "ABORTED during " + state;
                }
                // Close the gate once, so a volley can never leave it open. The intake goes back to
                // the OpMode's own buttons (it stops, unless the driver is holding intake).
                gateCommand = GateCommand.CLOSE;
                state = SequenceState.IDLE;
            }
            return;
        }

        // The button is held. Do the step we are on.
        switch (state) {
            case IDLE:
                // A new volley. Start the log over and close the gate.
                state = SequenceState.WAIT_FOR_READY;
                stepTimer.reset();
                volleyTimer.reset();
                lastLoopSeconds = 0.0;
                flywheelDipCount = 0;
                flywheelWasAtSpeed = false;
                pausedSeconds = 0.0;
                secondsToReady = -1.0;
                volleySeconds = -1.0;
                lastResult = "running";
                gateCommand = GateCommand.CLOSE;
                break;

            case WAIT_FOR_READY:
                // Keep the gate closed, so balls are held back while the wheels get ready.
                gateCommand = GateCommand.CLOSE;
                if (readyToStart) {
                    secondsToReady = volleyTimer.seconds();
                    state = SequenceState.FEED;
                    stepTimer.reset();
                    flywheelWasAtSpeed = flywheelAtSpeed;
                    setFeedOrders(flywheelAtSpeed, turretAimed);
                }
                break;

            case FEED:
                setFeedOrders(flywheelAtSpeed, turretAimed);
                if (!intakeWanted) {
                    pausedSeconds = pausedSeconds + loopSeconds;
                }

                // A drop out of "at speed" means a ball has just gone through.
                if (flywheelWasAtSpeed && !flywheelAtSpeed) {
                    flywheelDipCount = flywheelDipCount + 1;
                }
                flywheelWasAtSpeed = flywheelAtSpeed;

                if (maxFeedSeconds > NO_FEED_LIMIT && stepTimer.seconds() >= maxFeedSeconds) {
                    // A feed limit was set and it ran out. Close the gate. The intake goes back to
                    // the OpMode (it stops).
                    state = SequenceState.DONE;
                    lastResult = "feed limit reached (let go of the button)";
                    volleySeconds = volleyTimer.seconds();
                    gateCommand = GateCommand.CLOSE;
                    ownsIntake = false;
                    intakeWanted = false;
                    intakePauseReason = "";
                }
                break;

            case DONE:
                // Only with a feed limit. Stay closed until the button is released.
                gateCommand = GateCommand.CLOSE;
                break;
        }
    }

    // The orders while feeding: gate OPENED, and the intake runs when the flywheel is at speed and the
    // turret is aimed. When it is paused, remember why, so the screen can say.
    private void setFeedOrders(boolean flywheelAtSpeed, boolean turretAimed) {
        gateCommand = GateCommand.OPEN;
        ownsIntake = true;
        boolean flywheelOk = !INTAKE_WAITS_FOR_SPEED || flywheelAtSpeed;
        boolean aimOk = !INTAKE_WAITS_FOR_AIM || turretAimed;
        intakeWanted = flywheelOk && aimOk;
        if (!flywheelOk && !aimOk) {
            intakePauseReason = "flywheel not at speed AND turret not aimed";
        } else if (!flywheelOk) {
            intakePauseReason = "flywheel not at speed";
        } else if (!aimOk) {
            intakePauseReason = "turret not aimed";
        } else {
            intakePauseReason = "";
        }
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks what to do
    // ==================================================================

    public GateCommand getGateCommand() {
        return gateCommand;
    }

    // true = the sequence is in charge of the intake this loop (see wantsIntake()).
    public boolean ownsIntake() {
        return ownsIntake;
    }

    // Only means something when ownsIntake() is true: true = run the intake, false = stop it.
    public boolean wantsIntake() {
        return intakeWanted;
    }

    // Only means something when ownsIntake() is true and wantsIntake() is false: why the intake is
    // paused ("flywheel not at speed", "turret not aimed", or both). Empty when it is running.
    public String getIntakePauseReason() {
        return intakePauseReason;
    }

    public SequenceState getState() {
        return state;
    }

    // true from the moment the button is pressed until it is released.
    public boolean isRunning() {
        return state != SequenceState.IDLE;
    }

    // ------------------------------------------------------------------
    // The log of the last volley (for the test sheet and the screen)
    // ------------------------------------------------------------------
    // Seconds from the button press to the first ball being allowed to go. -1 if it never was.
    public double getSecondsToReady() {
        return secondsToReady;
    }

    // Seconds from the button press to the end of the feed. -1 if the volley never got to feed.
    public double getVolleySeconds() {
        return volleySeconds;
    }

    // How many times the flywheel dipped during the feed. About one per ball shot.
    public int getFlywheelDipCount() {
        return flywheelDipCount;
    }

    // How long the intake was paused during the feed (flywheel or turret), in seconds.
    public double getPausedSeconds() {
        return pausedSeconds;
    }

    public String getLastResult() {
        return lastResult;
    }

    // The feed limit in seconds, or NO_FEED_LIMIT (0) for "until released".
    public double getMaxFeedSeconds() {
        return maxFeedSeconds;
    }

    /*
     * Adds the sequence's own data to the telemetry, so the TeleOp and the test OpMode show the
     * same lines. The SDK sends it to the Driver Station after every loop.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- FIRING SEQUENCE ---");
        telemetry.addData("Sequence step", state);
        if (maxFeedSeconds > NO_FEED_LIMIT) {
            telemetry.addData("Feed limit", "%.1f s", maxFeedSeconds);
        } else {
            telemetry.addData("Feed limit", "none (feeds until the trigger is released)");
        }
        if (state == SequenceState.FEED) {
            telemetry.addData("Feeding for", "%.1f s   (intake paused %.1f s of it)", stepTimer.seconds(), pausedSeconds);
        }
        telemetry.addData("Flywheel dips this volley (about 1 per ball)", flywheelDipCount);
        telemetry.addData("Last volley", lastResult);
        if (secondsToReady >= 0.0) {
            telemetry.addData("Last volley: seconds to ready", "%.2f", secondsToReady);
        }
        if (volleySeconds >= 0.0) {
            telemetry.addData("Last volley: total seconds", "%.2f", volleySeconds);
        }
    }
}
