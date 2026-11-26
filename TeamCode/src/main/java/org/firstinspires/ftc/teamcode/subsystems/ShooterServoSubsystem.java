package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Shooter Servo Subsystem
 * Owns the shooter GATE servo (goBILDA dual mode, 300 degree torque servo).
 * Only this class touches that servo. OpModes call its methods.
 *
 * The gate has THREE places it can be:
 *   CLOSED -> the gate holds the balls back, so balls can be loaded into the shooter.
 *   OPENED -> the gate lets a ball pass into a flywheel and shooter that are ready.
 *   LIFTED -> the gate lifts to push the LAST ball into the shooter.
 *
 * goToClosed(), goToOpened() and goToLifted() pick one on purpose. toggle() flips between
 * CLOSED and OPENED only (from LIFTED it goes to CLOSED). The test OpModes that only need
 * "shut" and "open" use it.
 *
 * STATE (what we told the gate to do) is CLOSED, OPENED or LIFTED.
 * A servo cannot tell us where it really is, so there is no sensor STATUS here.
 * The closest thing is isAtPosition(): it is true once SERVO_TRAVEL_SECONDS have passed
 * since the last command, which is our guess that the gate has finished moving.
 *
 * CLOSING: every move, closing included, is instant. The servo goes to the new position as fast as
 * it can (its top speed, set by the servo type and how far it turns). We tried a slow, "gentle" close
 * (issue #63) and dropped it. It only helps if it is slower than the servo's own top speed, and the
 * coach wanted a faster close, so there was nothing left for it to do.
 *
 * CLEARING THE CHUTE: when the gate closes from OPENED or from LIFTED, a ball may be sitting in its
 * way. The OpMode runs the intake in REVERSE for a short time to push that ball back out.
 * isClearingChute() is true for CLOSE_REVERSE_SECONDS after such a close is commanded. A servo
 * cannot tell us when the gate is really shut, so this is a TIMER we tune on the robot, not a
 * measurement. Nothing needs update(): the gate only needs timers.
 * To FIND the right time, a test OpMode calls nudgeCloseReverseSeconds() (FiringSequenceAlpha: D-pad
 * Left / Right, 0.05 s per tap). Like nudgePosition(), a nudge only lasts while the OpMode runs, so
 * type the number you find into CLOSE_REVERSE_SECONDS.
 *
 * Robot configuration (on the Driver Station):
 *   Servo name: shooter_servo   (goBILDA dual mode servo, configured as a plain "Servo")
 * The dual mode servo must be set to POSITIONAL mode (not continuous) with the goBILDA
 * Servo Programmer. In continuous mode these position commands would spin it instead.
 */
public class ShooterServoSubsystem {

    // ------------------------------------------------------------------
    // Hardware name. This MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    private static final String SERVO_NAME = "shooter_servo";

    // ------------------------------------------------------------------
    // Gate positions
    // ------------------------------------------------------------------
    // A servo position is a number from 0.0 to 1.0. That is the servo's whole travel:
    // 0.0 is one end and 1.0 is the other end. This servo travels 300 degrees, so a
    // 90 degree gate move is about 0.30 (90 / 300). That is only a rough guide, because
    // the SDK's default pulse range is not exactly goBILDA's.
    //
    private static final double CLOSED_POSITION = 0.80;
    private static final double OPENED_POSITION = 0.50;
    private static final double LIFTED_POSITION = 0.00;

    // The servo does not tell us when it arrives. This is how long we assume a move takes.
    // (CLOSED to LIFTED is the whole travel, so it may need more time than a shorter move.)
    // Too short and isAtPosition() lies. Too long and we wait for nothing.
    // Nothing relies on this value aside from telemetry status.
    private static final double SERVO_TRAVEL_SECONDS = 0.50;

    // How many SECONDS the intake should run in REVERSE after the gate starts to close from OPENED or
    // from LIFTED, to push a ball out of the gate's way (see CLEARING THE CHUTE at the top). The
    // same time is used from both positions. The gate closes at the servo's top speed, so this is
    // only our guess for "long enough to clear the chute", not how long the gate takes.
    //   Bigger  = the intake reverses longer (more sure the chute is clear, slower to get going again).
    //   Smaller = a shorter reverse.
    // Find it at school with "Firing Sequence - Alpha": D-pad Left / Right changes this time by 0.05 s
    // while the OpMode runs. Leave a ball in the chute, fire, let go, and film the throat in slow
    // motion. Find the SHORTEST time that clears the ball every time and the LONGEST time before a ball
    // is pushed out the FRONT of the intake, then type a number a little above the shortest one here.
    // (The steps are in issue #116 and in biobuzz-test-and-match-log.md, section 5.)
    // on the robot with a ball left in the chute.
    // Ran Fruitport event at 0.50
    private static final double CLOSE_REVERSE_SECONDS = 0.25;

    // The most nudgeCloseReverseSeconds() can raise the reverse time to, in SECONDS. A long reverse
    // pushes balls out of the FRONT of the intake, so there is a limit. (It also stops a runaway nudge.)
    private static final double MAX_CLOSE_REVERSE_SECONDS = 1.0;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    // STATE = where we told the gate to go.
    public enum GateState {
        CLOSED,   // Gate is at (or moving to) the closed position.
        OPENED,   // Gate is at (or moving to) the opened position.
        LIFTED    // Gate is at (or moving to) the lifted position.
    }

    // ------------------------------------------------------------------
    // Hardware and data. Only this class changes these.
    // ------------------------------------------------------------------
    private Servo shooterServo;

    private GateState state = GateState.CLOSED;

    // These start at the constants above. They are variables (not constants) only so
    // nudgePosition() can change them while we are finding the right numbers.
    private double closedPosition = CLOSED_POSITION;
    private double openedPosition = OPENED_POSITION;
    private double liftedPosition = LIFTED_POSITION;

    // Starts at CLOSE_REVERSE_SECONDS. It is a variable (not a constant) only so
    // nudgeCloseReverseSeconds() can change it while we are finding the right time.
    // isClearingChute() reads THIS one, not the constant.
    private double closeReverseSeconds = CLOSE_REVERSE_SECONDS;

    // Times how long ago we last gave the servo a new position.
    private ElapsedTime travelTimer = new ElapsedTime();

    // true from the moment the gate is commanded to close from OPENED or LIFTED. It is only a flag:
    // isClearingChute() also checks the timer, so it goes false by itself.
    private boolean chuteClearing = false;
    private ElapsedTime chuteClearTimer = new ElapsedTime();

    // False until we send the servo its first position (in the constructor).
    private boolean servoHasBeenCommanded = false;

    /*
     * Setup. Gets the servo from the robot configuration and moves the gate
     * to the CLOSED position, so we always start from a known place.
     * (Rule R103.B allows an OpMode to pre-position servos at INIT.)
     */
    public ShooterServoSubsystem(HardwareMap hardwareMap) {
        // The name MUST match the robot configuration exactly.
        shooterServo = hardwareMap.get(Servo.class, SERVO_NAME);

        goToClosed();
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the gate what to do
    // ==================================================================

    /*
     * Move the gate to the CLOSED position.
     * Safe to call every loop. A repeat call does not restart the travel timer.
     */
    public void goToClosed() {
        setState(GateState.CLOSED);
    }

    /*
     * Move the gate to the OPENED position.
     * Safe to call every loop. A repeat call does not restart the travel timer.
     */
    public void goToOpened() {
        setState(GateState.OPENED);
    }

    /*
     * Move the gate to the LIFTED position.
     * Safe to call every loop. A repeat call does not restart the travel timer.
     */
    public void goToLifted() {
        setState(GateState.LIFTED);
    }

    /*
     * Flip the gate between CLOSED and OPENED. From LIFTED it goes to CLOSED.
     * Call this ONCE per button press. The OpMode has to notice the press,
     * because holding a button would flip it every loop.
     */
    public void toggle() {
        if (state == GateState.CLOSED) {
            goToOpened();
        } else {
            goToClosed();
        }
    }

    /*
     * For finding the right positions. Moves the position of the CURRENT state
     * (CLOSED, OPENED or LIFTED) by a small amount, and sends the gate there.
     * A positive amount moves toward 1.0, a negative amount toward 0.0.
     * The result is kept between 0.0 and 1.0.
     *
     * This only changes the numbers while the OpMode runs. When you find a good
     * number, you still have to type it into CLOSED_POSITION / OPENED_POSITION /
     * LIFTED_POSITION above.
     */
    public void nudgePosition(double amount) {
        if (state == GateState.CLOSED) {
            closedPosition = Range.clip(closedPosition + amount, 0.0, 1.0);
        } else if (state == GateState.OPENED) {
            openedPosition = Range.clip(openedPosition + amount, 0.0, 1.0);
        } else {
            liftedPosition = Range.clip(liftedPosition + amount, 0.0, 1.0);
        }
        sendServoToCurrentState();
    }

    /*
     * For finding how long the intake should reverse after the gate closes. Changes the time
     * isClearingChute() uses by this many SECONDS: a positive amount makes the reverse LONGER, a
     * negative amount makes it SHORTER. The result is kept between 0.0 and MAX_CLOSE_REVERSE_SECONDS.
     *
     * The time is rounded to two decimals so 0.05 steps stay clean on the screen (0.15, not
     * 0.15000000000000002), and so the number we read off the screen is exactly the one to type in.
     * A reverse that is already running uses the new time at once.
     *
     * This only changes the number while the OpMode runs. When you find a good time, you still
     * have to type it into CLOSE_REVERSE_SECONDS above.
     */
    public void nudgeCloseReverseSeconds(double amount) {
        double newSeconds = closeReverseSeconds + amount;
        newSeconds = Math.round(newSeconds * 100.0) / 100.0;
        closeReverseSeconds = Range.clip(newSeconds, 0.0, MAX_CLOSE_REVERSE_SECONDS);
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks the gate what is happening
    // ==================================================================

    public GateState getState() {
        return state;
    }

    public boolean isClosed() {
        return state == GateState.CLOSED;
    }

    public boolean isOpened() {
        return state == GateState.OPENED;
    }

    public boolean isLifted() {
        return state == GateState.LIFTED;
    }

    /*
     * True once enough time has passed for the gate to finish its move.
     * This is a TIME GUESS, not a sensor reading. This is the gate's half of the
     * "OK to feed a ball" check, once we build that.
     */
    public boolean isAtPosition() {
        return travelTimer.seconds() >= SERVO_TRAVEL_SECONDS;
    }

    /*
     * True for the reverse time (CLOSE_REVERSE_SECONDS, unless nudgeCloseReverseSeconds() changed it)
     * after the gate is commanded to close from OPENED or LIFTED.
     * The OpMode reverses the intake while this is true. It is false when the gate was already
     * closed, and it goes false at once if the gate is told to open or lift. A TIME GUESS.
     */
    public boolean isClearingChute() {
        return chuteClearing && chuteClearTimer.seconds() < closeReverseSeconds;
    }

    // How many SECONDS the intake reverses after the gate closes right now (CLOSE_REVERSE_SECONDS,
    // or the nudged number). This is the number to type into CLOSE_REVERSE_SECONDS.
    public double getCloseReverseSeconds() {
        return closeReverseSeconds;
    }

    // The position (0.0 to 1.0) we are currently sending to the servo.
    public double getTargetPosition() {
        if (state == GateState.CLOSED) {
            return closedPosition;
        }
        if (state == GateState.OPENED) {
            return openedPosition;
        }
        return liftedPosition;
    }

    /*
     * Adds the gate's own data to the telemetry. The SDK sends telemetry to the
     * Driver Station after every loop(), so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- SHOOTER GATE ---");
        telemetry.addData("Gate State", state);
        telemetry.addData("Gate Position", "%.3f", getTargetPosition());
        telemetry.addData("Clearing chute (intake reverse)?", isClearingChute());
        telemetry.addData("Intake reverse after close", "%.2f s", closeReverseSeconds);
        telemetry.addData("Gate At Position?", isAtPosition());
        telemetry.addData("Closed Position", "%.3f", closedPosition);
        telemetry.addData("Opened Position", "%.3f", openedPosition);
        telemetry.addData("Lifted Position", "%.3f", liftedPosition);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    // Change the state and send the servo there. If we are ALREADY in that state, do
    // nothing, so a held button will not keep restarting the travel timer (or the chute timer).
    private void setState(GateState newState) {
        // The very first call (from the constructor) must always go through, even though
        // the state already says CLOSED. servoHasBeenCommanded is false until then.
        if (servoHasBeenCommanded && newState == state) {
            return;
        }
        // Closing from OPENED or LIFTED starts the intake-reverse window. Any other move ends it.
        boolean startsClosing = servoHasBeenCommanded && newState == GateState.CLOSED;
        chuteClearing = startsClosing;
        if (startsClosing) {
            chuteClearTimer.reset();
        }
        state = newState;
        sendServoToCurrentState();
    }

    private void sendServoToCurrentState() {
        shooterServo.setPosition(getTargetPosition());
        servoHasBeenCommanded = true;
        travelTimer.reset();
    }
}
