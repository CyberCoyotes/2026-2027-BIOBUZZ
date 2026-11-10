package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Firing Sequence
 * The steps of one "volley": wait until we are ready, open the gate, feed the balls one at a time,
 * and close the gate. It is the same sequence the Auto
 * (ShootAndParkAuto) works out for itself, so the TeleOp and the Auto can share one copy.
 *
 * WHAT IT IS (and is not)
 *   This is a plain class with NO hardware and NO gamepad. It is not a mechanism, so its name does
 *   not end in "Subsystem" (like ShotTable and HiveBearingMemory). It does not move anything. Each
 *   loop the OpMode tells it three yes/no answers, and it answers with a gate command and whether
 *   the intake should run. The OpMode then gives those orders to ShooterServoSubsystem and
 *   IntakeSubsystem, which still own their hardware.
 *
 * HOW TO USE IT (once per loop, AFTER shooterSubsystem.update(), because the answers must be fresh)
 *   firingSequence.update(fireHeld, readyToStart, flywheelAtSpeed);
 *     fireHeld       - is the fire button held right now? (The OpMode reads the gamepad.)
 *     readyToStart   - may the first ball go? The OpMode decides: the flywheel at speed, the turret
 *                      on target, and (when the speed comes from the shot table) the camera sees the
 *                      hive right now.
 *     flywheelAtSpeed- is the flywheel at speed right now? (ShooterSubsystem.isFlywheelAtSpeed().)
 *                      It paces the feed, and counts the dips.
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
 *   2. FEED           : gate OPENED, for FEED_SECONDS. The intake runs only while the flywheel is at
 *                       speed, so balls go one at a time: each ball slows the flywheel for a moment
 *                       and the intake pauses until it recovers (INTAKE_WAITS_FOR_SPEED).
 *   3. DONE           : gate CLOSED, intake not mine. Stays here until the button is released, so a
 *                       held button can NOT fire an empty chute over and over. To fire again,
 *                       release and press again.
 *   ABORT: releasing the button at any step closes the gate once and gives the intake back. Any
 *   balls not yet shot stay in the chute.
 *
 * NO LIFT STEP (removed, issue #63): there used to be a step after FEED that LIFTED the gate for half a
 * second with the intake running, to push the last ball in. Its timing was off and the last ball got
 * wedged, so it is gone: when the feed time is up, the gate just CLOSES. (The manual LIFTED gate
 * button between volleys is still there, and so is ShooterServoSubsystem.goToLifted().) A ball that is
 * half way through when the gate closes could still jam, so watch for it in the robot test.
 *
 * WHY THE GATE STAYS OPEN DURING THE FEED
 *   Opening and closing it for every ball would add the servo's travel time (about half a second
 *   each way, a guess) to every ball. The flywheel dip after each ball does the pacing instead.
 *
 * NO BALL SENSOR: we cannot count balls. FEED_SECONDS is a time. The dip counter (about one dip
 * per ball shot) is there to check that. If the dips match the balls fired, a later version can end
 * the feed early when the dips stop. Practice with FEED_SECONDS until all 4 balls go, then keep it
 * the same for matches.
 */
public class FiringSequence {

    // ------------------------------------------------------------------
    // Tunable numbers (these came from ShootAndParkAuto's first values)
    // ------------------------------------------------------------------
    // How long the gate stays OPENED while the intake feeds balls, in seconds. The intake only runs
    // while the flywheel is at speed, so this is the time the feed is ALLOWED to run, not the time
    // the intake is on. Four balls with a recovery after each one take a few seconds.
    // Starting guess. Tune on the robot until all 4 balls go.
    private static final double FEED_SECONDS = 3.0;

    // true  = during the feed, the intake runs only while the flywheel is at speed (one ball at a
    //         time).
    // false = the intake runs the whole feed no matter what. Use this to check that the intake
    //         moves the balls through the chute at all.
    private static final boolean INTAKE_WAITS_FOR_SPEED = true;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    // STATE = which step the volley is on.
    public enum SequenceState {
        IDLE,            // No volley. The fire button is not held.
        WAIT_FOR_READY,  // Button held. Gate CLOSED. Waiting for readyToStart.
        FEED,            // Gate OPENED. Feeding balls, one at a time.
        DONE             // Finished. Gate CLOSED. Waiting for the button to be released.
    }

    // What the OpMode should tell the gate this loop.
    public enum GateCommand {
        NONE,   // Not my job this loop. Use the manual buttons.
        CLOSE,
        OPEN
    }

    private SequenceState state = SequenceState.IDLE;

    // The orders for THIS loop. update() works them out again every loop.
    private GateCommand gateCommand = GateCommand.NONE;
    private boolean ownsIntake = false;
    private boolean intakeWanted = false;

    // Times the current step, and the whole volley.
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime volleyTimer = new ElapsedTime();

    // The flywheel dip count. A dip is a loop where the flywheel was at speed last loop and is not
    // now. Each ball that goes through slows the flywheel, so this is about one per ball.
    private boolean flywheelWasAtSpeed = false;
    private int flywheelDipCount = 0;

    // The log of the last volley. They stay until the next volley starts. -1 means "has not
    // happened (yet)".
    private double secondsToReady = -1.0;
    private double volleySeconds = -1.0;
    private String lastResult = "none yet";

    // ==================================================================
    // ACTION: the OpMode calls this once per loop
    // ==================================================================

    /*
     * Call this ONCE per loop. See the top of this file for what the three answers mean. After it,
     * ask getGateCommand(), ownsIntake() and wantsIntake().
     */
    public void update(boolean fireHeld, boolean readyToStart, boolean flywheelAtSpeed) {
        // The orders are only for this loop. Start from "not my job".
        gateCommand = GateCommand.NONE;
        ownsIntake = false;
        intakeWanted = false;

        // The button is not held: no volley. If one was running, it is over (finished or aborted).
        if (!fireHeld) {
            if (state != SequenceState.IDLE) {
                if (state == SequenceState.DONE) {
                    lastResult = "finished";
                } else {
                    // Let go before the end. (If a jam was cleared mid-volley, the feed time kept
                    // running, so let go and press again to fire the balls that are left.)
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
                flywheelDipCount = 0;
                flywheelWasAtSpeed = false;
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
                    setFeedOrders(flywheelAtSpeed);
                }
                break;

            case FEED:
                setFeedOrders(flywheelAtSpeed);

                // A drop out of "at speed" means a ball has just gone through.
                if (flywheelWasAtSpeed && !flywheelAtSpeed) {
                    flywheelDipCount = flywheelDipCount + 1;
                }
                flywheelWasAtSpeed = flywheelAtSpeed;

                if (stepTimer.seconds() >= FEED_SECONDS) {
                    // The feed time is up. Done: close the gate. The intake goes back to the OpMode
                    // (it stops).
                    state = SequenceState.DONE;
                    lastResult = "finished (let go of the button)";
                    volleySeconds = volleyTimer.seconds();
                    gateCommand = GateCommand.CLOSE;
                    ownsIntake = false;
                    intakeWanted = false;
                }
                break;

            case DONE:
                // Stay closed until the button is released.
                gateCommand = GateCommand.CLOSE;
                break;
        }
    }

    // The orders while feeding: gate OPENED, and the intake runs when the flywheel is at speed.
    private void setFeedOrders(boolean flywheelAtSpeed) {
        gateCommand = GateCommand.OPEN;
        ownsIntake = true;
        intakeWanted = !INTAKE_WAITS_FOR_SPEED || flywheelAtSpeed;
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

    // Seconds from the button press to the end of the feed. -1 if the volley did not finish.
    public double getVolleySeconds() {
        return volleySeconds;
    }

    // How many times the flywheel dipped during the feed. About one per ball shot.
    public int getFlywheelDipCount() {
        return flywheelDipCount;
    }

    public String getLastResult() {
        return lastResult;
    }

    public double getFeedSeconds() {
        return FEED_SECONDS;
    }

    /*
     * Adds the sequence's own data to the telemetry, so the TeleOp and the test OpMode show the
     * same lines. The SDK sends it to the Driver Station after every loop.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- FIRING SEQUENCE ---");
        telemetry.addData("Sequence step", state);
        telemetry.addData("Feed seconds", "%.1f", FEED_SECONDS);
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
