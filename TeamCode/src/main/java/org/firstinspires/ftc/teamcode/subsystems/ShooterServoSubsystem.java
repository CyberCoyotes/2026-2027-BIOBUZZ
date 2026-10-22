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
 * The gate has two places it can be:
 *   LOADING  -> the gate is where a ball can be loaded.
 *   SHOOTING -> the gate is where the ball can be fed into the shooter.
 * It moves about 90 degrees between the two.
 *
 * toggle() flips between them. goToLoading() and goToShooting() pick one on purpose.
 *
 * STATE (what we told the gate to do) is LOADING or SHOOTING.
 * A servo cannot tell us where it really is, so there is no sensor STATUS here.
 * The closest thing is isAtPosition(): it is true once SERVO_TRAVEL_SECONDS have passed
 * since the last command, which is our guess that the gate has finished moving.
 *
 * Nothing calls update() for this class, because there is no ongoing work.
 * A timer is all the gate needs, and isAtPosition() reads that timer itself.
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
    // TBD: these two numbers are PLACEHOLDERS. We have not measured the real gate yet.
    // Find them with ShooterServoAlpha / the Shooter Alpha OpModes: the D-pad moves the
    // current position by a small step, so you can creep the gate to the right spot
    // (the OpMode shows the number). Then type the numbers you found in here.
    //
    // They start close together on purpose, so the first test cannot slam the gate.
    private static final double LOADING_POSITION = 0.30;
    // 0.0 is the end of the servo's travel. A servo cannot go below it.
    private static final double SHOOTING_POSITION = 0.0;

    // The servo does not tell us when it arrives. This is how long we assume a 90 degree
    // move takes. Too short and isAtPosition() lies. Too long and we wait for nothing.
    // Starting guess. Time the gate with a phone stopwatch, then change this number.
    private static final double SERVO_TRAVEL_SECONDS = 0.50;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    // STATE = where we told the gate to go.
    public enum GateState {
        LOADING,   // Gate is at (or moving to) the loading position.
        SHOOTING   // Gate is at (or moving to) the shooting position.
    }

    // ------------------------------------------------------------------
    // Hardware and data. Only this class changes these.
    // ------------------------------------------------------------------
    private Servo shooterServo;

    private GateState state = GateState.LOADING;

    // These start at the constants above. They are variables (not constants) only so
    // nudgePosition() can change them while we are finding the right numbers.
    private double loadingPosition = LOADING_POSITION;
    private double shootingPosition = SHOOTING_POSITION;

    // Times how long ago we last gave the servo a new position.
    private ElapsedTime travelTimer = new ElapsedTime();

    // False until we send the servo its first position (in the constructor).
    private boolean servoHasBeenCommanded = false;

    /*
     * Setup. Gets the servo from the robot configuration and moves the gate
     * to the LOADING position, so we always start from a known place.
     * (Rule R103.B allows an OpMode to pre-position servos at INIT.)
     */
    public ShooterServoSubsystem(HardwareMap hardwareMap) {
        // The name MUST match the robot configuration exactly.
        shooterServo = hardwareMap.get(Servo.class, SERVO_NAME);

        goToLoading();
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the gate what to do
    // ==================================================================

    /*
     * Move the gate to the LOADING position.
     * Safe to call every loop. A repeat call does not restart the travel timer.
     */
    public void goToLoading() {
        setState(GateState.LOADING);
    }

    /*
     * Move the gate to the SHOOTING position.
     * Safe to call every loop. A repeat call does not restart the travel timer.
     */
    public void goToShooting() {
        setState(GateState.SHOOTING);
    }

    /*
     * Flip the gate to the OTHER position.
     * Call this ONCE per button press. The OpMode has to notice the press
     * (see ShooterServoAlpha), because holding a button would flip it every loop.
     */
    public void toggle() {
        if (state == GateState.LOADING) {
            goToShooting();
        } else {
            goToLoading();
        }
    }

    /*
     * For finding the right positions. Moves the position of the CURRENT state
     * (LOADING or SHOOTING) by a small amount, and sends the gate there.
     * A positive amount moves toward 1.0, a negative amount toward 0.0.
     * The result is kept between 0.0 and 1.0.
     *
     * This only changes the numbers while the OpMode runs. When you find a good
     * number, you still have to type it into LOADING_POSITION / SHOOTING_POSITION above.
     */
    public void nudgePosition(double amount) {
        if (state == GateState.LOADING) {
            loadingPosition = Range.clip(loadingPosition + amount, 0.0, 1.0);
        } else {
            shootingPosition = Range.clip(shootingPosition + amount, 0.0, 1.0);
        }
        sendServoToCurrentState();
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks the gate what is happening
    // ==================================================================

    public GateState getState() {
        return state;
    }

    public boolean isLoading() {
        return state == GateState.LOADING;
    }

    public boolean isShooting() {
        return state == GateState.SHOOTING;
    }

    /*
     * True once enough time has passed for the gate to finish its move.
     * This is a TIME GUESS, not a sensor reading. This is the gate's half of the
     * "OK to feed a ball" check, once we build that.
     */
    public boolean isAtPosition() {
        return travelTimer.seconds() >= SERVO_TRAVEL_SECONDS;
    }

    // The position (0.0 to 1.0) we are currently sending to the servo.
    public double getTargetPosition() {
        if (state == GateState.LOADING) {
            return loadingPosition;
        }
        return shootingPosition;
    }

    /*
     * Adds the gate's own data to the telemetry. The SDK sends telemetry to the
     * Driver Station after every loop(), so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- SHOOTER GATE ---");
        telemetry.addData("Gate State", state);
        telemetry.addData("Gate Position", "%.3f", getTargetPosition());
        telemetry.addData("Gate At Position?", isAtPosition());
        telemetry.addData("Loading Position", "%.3f", loadingPosition);
        telemetry.addData("Shooting Position", "%.3f", shootingPosition);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    // Change the state and send the servo there. If we are ALREADY in that state, do
    // nothing, so a held button will not keep restarting the travel timer.
    private void setState(GateState newState) {
        // The very first call (from the constructor) must always go through, even though
        // the state already says LOADING. servoHasBeenCommanded is false until then.
        if (servoHasBeenCommanded && newState == state) {
            return;
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
