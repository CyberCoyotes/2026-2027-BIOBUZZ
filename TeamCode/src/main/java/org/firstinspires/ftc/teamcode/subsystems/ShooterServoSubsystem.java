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
 * GENTLE CLOSE (issue #63): goToClosed() sends the servo straight to CLOSED, as fast as the servo
 * goes. If a ball is in the way (a single ball left when the feed ends), a fast close can slam it
 * or wedge it. goToClosedGently() instead walks the servo toward CLOSED slowly
 * (GENTLE_CLOSE_SPEED_POSITION_PER_SECOND), so the gate pushes the ball out of the way instead of
 * slamming into it. A servo pushes harder the further it is from the position it was told to
 * hold, so a slow walk is also a gentle push. goToOpened(), goToLifted() and goToClosed() are all
 * still instant, and any of them cancels a gentle close that is still going.
 *
 * update() IS NEEDED ONLY FOR goToClosedGently(). The OpMode must call update() ONCE per loop(), or
 * a gentle close never moves. (Everything else does not need update(): a timer is all the gate
 * needs, and isAtPosition() reads that timer itself.)
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
    // TBD: these three numbers are still being tuned. Find them with the Shooter Alpha OpModes:
    // the D-pad moves the current position by a small step, so you can creep the gate to the
    // right spot (the OpMode shows the number). Then type the numbers you found in here.
    // (They used to be LOADING 0.30 and SHOOTING 0.0, before the gate had a third position.)
    private static final double CLOSED_POSITION = 0.80;
    private static final double OPENED_POSITION = 0.50;
    private static final double LIFTED_POSITION = 0.00;

    // The servo does not tell us when it arrives. This is how long we assume a move takes.
    // (CLOSED to LIFTED is the whole travel, so it may need more time than a shorter move.)
    // Too short and isAtPosition() lies. Too long and we wait for nothing.
    // Starting guess. Time the gate with a phone stopwatch, then change this number.
    private static final double SERVO_TRAVEL_SECONDS = 0.50;

    // How fast the gate walks toward CLOSED during a GENTLE close, in servo position units per
    // second (the whole travel is 1.0, so 0.40 is 40% of the travel per second). OPENED to CLOSED
    // is 0.30 apart, so at 0.40 it takes about 0.75 seconds.
    //   Smaller = slower and gentler, but the gate takes longer to shut.
    //   Bigger  = quicker, and closer to the old instant close (a very big number is the same).
    // Starting guess. Tune on the robot with one ball left in the chute: it should be pushed out of
    // the way, not slammed or wedged.
    private static final double GENTLE_CLOSE_SPEED_POSITION_PER_SECOND = 0.40;

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

    // Times how long ago we last gave the servo a new position.
    private ElapsedTime travelTimer = new ElapsedTime();

    // The position we are really sending to the servo right now (0.0 to 1.0). It equals the
    // state's position, except DURING a gentle close, when it is on its way there.
    private double commandedPosition = CLOSED_POSITION;

    // true while a gentle close is still walking toward CLOSED. update() moves it along.
    private boolean closingGently = false;
    private ElapsedTime gentleCloseTimer = new ElapsedTime();

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
     * Move the gate to the CLOSED position SLOWLY, so a ball in the way is pushed aside instead of
     * slammed (see GENTLE CLOSE at the top). Needs update() every loop. Safe to call every loop:
     * a repeat call while the gate is closed, or already closing, does nothing. If the gate is
     * already closed it stays put. goToClosed() (instant) cancels a gentle close.
     */
    public void goToClosedGently() {
        if (!servoHasBeenCommanded) {
            // The very first command has nothing to walk from. Go straight there.
            goToClosed();
            return;
        }
        if (state == GateState.CLOSED) {
            // Already closed, or already closing gently. Do not restart anything.
            return;
        }
        state = GateState.CLOSED;
        closingGently = true;
        gentleCloseTimer.reset();
        travelTimer.reset();
        // commandedPosition stays where the gate is now. update() walks it toward CLOSED.
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
     * Call this ONCE per loop() if the OpMode uses goToClosedGently(). It walks a gentle close along:
     * each loop the servo is sent a little closer to CLOSED, at GENTLE_CLOSE_SPEED_POSITION_PER_SECOND.
     * It does nothing when no gentle close is going on, so it is always safe to call.
     */
    public void update() {
        if (!closingGently) {
            return;
        }

        // How far we may move this loop: speed x the time since the last loop.
        double secondsSinceLastStep = gentleCloseTimer.seconds();
        gentleCloseTimer.reset();
        double maxStep = GENTLE_CLOSE_SPEED_POSITION_PER_SECOND * secondsSinceLastStep;

        double distanceLeft = closedPosition - commandedPosition;
        if (Math.abs(distanceLeft) <= maxStep) {
            // This step gets us all the way there.
            commandedPosition = closedPosition;
            closingGently = false;
            travelTimer.reset();
        } else if (distanceLeft > 0.0) {
            commandedPosition = commandedPosition + maxStep;
        } else {
            commandedPosition = commandedPosition - maxStep;
        }
        shooterServo.setPosition(commandedPosition);
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
        return !closingGently && travelTimer.seconds() >= SERVO_TRAVEL_SECONDS;
    }

    // true while a gentle close is still on its way to CLOSED.
    public boolean isClosingGently() {
        return closingGently;
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
        if (closingGently) {
            telemetry.addData("Gate closing gently", "now at %.3f, going to %.3f", commandedPosition, closedPosition);
        }
        telemetry.addData("Gate At Position?", isAtPosition());
        telemetry.addData("Closed Position", "%.3f", closedPosition);
        telemetry.addData("Opened Position", "%.3f", openedPosition);
        telemetry.addData("Lifted Position", "%.3f", liftedPosition);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    // Change the state and send the servo there. If we are ALREADY in that state, do
    // nothing, so a held button will not keep restarting the travel timer.
    private void setState(GateState newState) {
        // The very first call (from the constructor) must always go through, even though
        // the state already says CLOSED. servoHasBeenCommanded is false until then.
        // A gentle close that is still going is NOT "already there": an instant command must
        // finish the job, or the gate would stay half way.
        if (servoHasBeenCommanded && newState == state && !closingGently) {
            return;
        }
        state = newState;
        sendServoToCurrentState();
    }

    private void sendServoToCurrentState() {
        // An instant command always ends a gentle close.
        closingGently = false;
        commandedPosition = getTargetPosition();
        shooterServo.setPosition(commandedPosition);
        servoHasBeenCommanded = true;
        travelTimer.reset();
    }
}
