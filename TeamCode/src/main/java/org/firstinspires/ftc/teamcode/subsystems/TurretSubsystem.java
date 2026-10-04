package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Turret Subsystem
 * Owns the motor that turns the shooter hood left and right. Only this class touches
 * the turret motor, the turret encoder, and the home switch. OpModes call its methods.
 *
 * ANGLE CONVENTION (looking down from above, the robot front / intake is at the top):
 *
 *                      0 deg   <- the HOME SWITCH, pointing at the intake
 *             +90 deg (left)     -90 deg (right)
 *               dead zone behind the robot: about +/-135 to 180 deg
 *
 *   - Angles are SIGNED. Left (counterclockwise) is POSITIVE. Right (clockwise) is NEGATIVE.
 *   - That matches how Pinpoint and Pedro Pathing measure the robot's heading, so later:
 *         shooter's field heading = robot heading + turret angle
 *   - POSITIVE motor power must turn the turret toward POSITIVE angles (left).
 *
 * HOW THE TURRET KNOWS WHERE IT IS
 *   A REV Through Bore Encoder rides on a pinion that meshes with the turret's ring gear.
 *   It only counts movement, so it needs a reference. The REV magnetic limit switch is
 *   that reference:
 *     1. Before the match, place the turret by hand ON the switch (0 degrees).
 *        If it starts on the switch, we zero right away (see setUpHardware).
 *     2. During the match, every time the turret rolls onto the switch, update() resets
 *        the angle to 0 so any drift is corrected. To stop this from firing over and
 *        over while the turret hovers at the magnet's edge, it only re-arms after the
 *        turret has moved REARM_DISTANCE_DEGREES away from home.
 *   We never reset the encoder just because an OpMode starts. The Control Hub keeps its
 *   counts between Auto and TeleOp, so the turret keeps its angle when TeleOp begins.
 *   isZeroConfirmed() is false until the switch has been seen at least once.
 *   Nothing here moves the turret at INIT, because the rules say the robot must not move
 *   after initialization (G304, G403).
 *
 * TWO WAYS TO MOVE THE TURRET
 *   1. POSITION: setTargetAngle(degrees). A small controller turns the turret there.
 *   2. MANUAL: setManualPower(power). The driver steers it with a stick.
 *   Either way the turret will not be driven past its soft limits.
 *
 * STATE (what we told it to do) is IDLE, MANUAL, or POSITION.
 * STATUS (what the sensors say) is getAngleDegrees(), isHomeSwitchActive(), isOnTarget().
 *
 * The OpMode must call update() ONCE per loop(). Nothing calls it for us.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name:           turret_motor        (goBILDA motor; NO encoder cable needed)
 *   Motor name:           turret_encoder      (REV Through Bore Encoder plugged into a motor
 *                                              port's encoder connector. No motor is attached
 *                                              and we never send it power. Configure it as a motor.)
 *   Digital Device name:  turret_home_switch  (REV magnetic limit switch. Configure it as a
 *                                              "Digital Device", NOT "REV Touch Sensor",
 *                                              because this class reads it as a DigitalChannel.)
 */
public class TurretSubsystem {

    // ------------------------------------------------------------------
    // Hardware names. These MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    private static final String TURRET_MOTOR_NAME = "turret_motor";
    private static final String TURRET_ENCODER_NAME = "turret_encoder";
    private static final String HOME_SWITCH_NAME = "turret_home_switch";

    // ------------------------------------------------------------------
    // Directions. These two MUST be right or the turret will run away!
    // ------------------------------------------------------------------
    // Positive power must turn the turret LEFT (counterclockwise, toward positive angles).
    // If positive power turns it right, flip this between FORWARD and REVERSE.
    private static final DcMotor.Direction TURRET_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;

    // Turning the turret LEFT must make the angle go UP.
    // If it goes down, flip this between FORWARD and REVERSE.
    private static final DcMotor.Direction TURRET_ENCODER_DIRECTION = DcMotor.Direction.FORWARD;

    // ------------------------------------------------------------------
    // Gear and encoder numbers: how encoder ticks turn into turret degrees
    // ------------------------------------------------------------------
    // The REV Through Bore Encoder makes 2048 cycles per turn. The Control Hub counts all
    // four edges of each cycle, so we see 8192 ticks per encoder turn.
    private static final double ENCODER_TICKS_PER_REV = 8192.0;

    // The encoder pinion and the ring gear mesh, so the encoder turns more than the turret does.
    // [CONFIRM] The ring gear is believed to be 48 teeth. Count it in Onshape. Both pinions are 16.
    private static final double RING_GEAR_TEETH = 48.0;
    private static final double ENCODER_PINION_TEETH = 16.0;

    private static final double DEGREES_PER_REV = 360.0;

    // Encoder ticks for ONE degree of turret turn. With 48 and 16 teeth this is about 68.3.
    // Check it on the robot: turn the turret by hand through a marked 90 degrees and see
    // that the angle on screen reads 90. If not, check the tooth counts above.
    private static final double TICKS_PER_TURRET_DEGREE =
            ENCODER_TICKS_PER_REV * (RING_GEAR_TEETH / ENCODER_PINION_TEETH) / DEGREES_PER_REV;

    // ------------------------------------------------------------------
    // Home switch
    // ------------------------------------------------------------------
    // The angle of the home switch. 0 means the switch is at the turret's center (pointing
    // at the intake). If you ever move the switch, change this.
    private static final double HOME_ANGLE_DEGREES = 0.0;

    // The REV magnetic limit switch is ACTIVE-LOW: it reads false when a magnet is near.
    // So "active" (turret is on the switch) means getState() == false.
    // If the Home Switch line in telemetry looks backwards, flip this to true.
    private static final boolean HOME_SWITCH_ACTIVE_STATE = false;

    // After a re-zero, the turret must move this far from home before the next re-zero.
    // That stops the angle from jumping around while the turret sits at the magnet's edge.
    private static final double REARM_DISTANCE_DEGREES = 10.0;

    // ------------------------------------------------------------------
    // Soft limits: the turret will not be driven past these angles.
    // ------------------------------------------------------------------
    // The full travel is about 270 degrees, which is +/-135. These start SMALL on purpose,
    // so a mistake in the first tests cannot hurt the cables or the frame.
    // After the encoder direction and the switch both check out, widen them to about +/-130
    // (a few degrees inside the real hard stops).
    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    // ------------------------------------------------------------------
    // Position control settings. All of these are starting guesses to tune on the robot.
    // ------------------------------------------------------------------
    // P: motor power per degree of error. 0.02 means 30 degrees of error asks for 60% power.
    // Raise it if the turret is slow to get there. Lower it if it overshoots or shakes.
    private static final double KP = 0.02;

    // D: slows the push as the turret speeds up, to calm overshoot.
    // Leave at 0.0 until the turret overshoots or oscillates. Then try a small number like 0.001.
    private static final double KD = 0.0;

    // A geared turret sticks a little. This extra power gets it started moving.
    // If the turret stops short of the target, raise it. If it jerks, lower it.
    private static final double FRICTION_POWER = 0.05;

    // The most power position control will use. Start low. Raise it once everything checks out.
    private static final double MAX_POWER = 0.5;

    // The most power the driver's stick can ask for in MANUAL mode.
    private static final double MAX_MANUAL_POWER = 0.4;

    // The turret counts as "on target" when it is within this many degrees of the target...
    // The hive window forgives about +/-5 to 13 degrees, so this can be fairly loose.
    private static final double ON_TARGET_TOLERANCE_DEGREES = 1.5;

    // ...for at least this long. This stops the answer flashing while it passes the target.
    private static final double ON_TARGET_HOLD_SECONDS = 0.1;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    // STATE = what we told the turret to do.
    public enum TurretState {
        IDLE,      // Not driving the turret. It is free to turn by hand.
        MANUAL,    // Running at a power the driver asked for.
        POSITION   // Turning to, and holding, a target angle.
    }

    // ------------------------------------------------------------------
    // Hardware and data. Only this class changes these.
    // ------------------------------------------------------------------
    private DcMotorEx turretMotor;
    private DcMotorEx turretEncoder;
    private DigitalChannel homeSwitch;

    private TurretState state = TurretState.IDLE;

    private double targetAngleDegrees = 0.0;
    private double manualPower = 0.0;

    // What the sensors reported the last time update() ran.
    private int encoderTicks = 0;
    private double angleDegrees = 0.0;
    private double previousAngleDegrees = 0.0;
    private double angularVelocityDegreesPerSecond = 0.0;

    // Home switch bookkeeping.
    private boolean homeSwitchWasActive = false;
    private boolean zeroConfirmed = false;
    private boolean rearmed = true;

    // For telemetry: the angle the turret THOUGHT it was at when it reached or left the switch.
    // When it reaches the switch, this shows how far off (drift) the angle was before the re-zero.
    // These two numbers also show whether the switch triggers at a different angle
    // depending on which way the turret is moving.
    private double angleWhenReachedSwitch = 0.0;
    private double angleWhenLeftSwitch = 0.0;

    // "On target" bookkeeping.
    private ElapsedTime onTargetTimer = new ElapsedTime();
    private boolean wasWithinTolerance = false;
    private boolean onTarget = false;

    // Times each loop, so we can work out how fast the turret is turning.
    private ElapsedTime loopTimer = new ElapsedTime();

    // True once we have asked the motor to hold position with BRAKE.
    private boolean brakeOn = false;

    /*
     * Setup. Gets the motor, the encoder, and the home switch from the robot configuration.
     * The turret starts in IDLE, free to turn by hand.
     */
    public TurretSubsystem(HardwareMap hardwareMap) {
        // The names MUST match the robot configuration exactly.
        turretMotor = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);
        turretEncoder = hardwareMap.get(DcMotorEx.class, TURRET_ENCODER_NAME);
        homeSwitch = hardwareMap.get(DigitalChannel.class, HOME_SWITCH_NAME);

        setUpHardware();
    }

    /*
     * Sets directions and modes, then works out the starting angle.
     */
    private void setUpHardware() {
        turretMotor.setDirection(TURRET_MOTOR_DIRECTION);
        turretEncoder.setDirection(TURRET_ENCODER_DIRECTION);

        homeSwitch.setMode(DigitalChannel.Mode.INPUT);

        // We control the motor with raw power. The turret's real angle comes from the
        // external encoder, so the motor's own encoder is not used for position.
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0.0);

        // The encoder only counts. It is not a motor, so we never give it power.
        // NOTE: we do NOT reset it here. The Control Hub keeps its count between Auto and
        // TeleOp, so the turret keeps its angle when the next OpMode starts.
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // FLOAT until someone commands the turret, so it can be turned by hand during INIT
        // to sit on the switch. Once we command it, we switch to BRAKE (see useBrake).
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brakeOn = false;

        // If the turret was placed on the switch, we know exactly where it is: 0 degrees.
        homeSwitchWasActive = isHomeSwitchActive();
        if (homeSwitchWasActive) {
            zeroAtHome();
        }

        encoderTicks = turretEncoder.getCurrentPosition();
        angleDegrees = ticksToAngleDegrees(encoderTicks);
        previousAngleDegrees = angleDegrees;
        loopTimer.reset();
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the turret what to do
    // ==================================================================

    /*
     * Turn the turret to an angle and hold it there. Positive is LEFT, negative is RIGHT,
     * 0 is straight ahead (the intake direction). The angle is held inside the soft limits.
     *
     * Safe to call every loop with the same angle. A repeat call changes nothing.
     */
    public void setTargetAngle(double degrees) {
        double newTargetAngle = Range.clip(degrees, MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

        // Same request as last time? Then there is nothing to do.
        if (state == TurretState.POSITION && newTargetAngle == targetAngleDegrees) {
            return;
        }

        useBrake();
        targetAngleDegrees = newTargetAngle;
        wasWithinTolerance = false;
        onTarget = false;
        state = TurretState.POSITION;
    }

    /*
     * Hold the turret wherever it is right now. Use this when the driver lets go of the stick.
     */
    public void holdCurrentAngle() {
        setTargetAngle(angleDegrees);
    }

    /*
     * Drive the turret at a power. Use -1.0 to 1.0, where 1.0 means MAX_MANUAL_POWER.
     * Positive turns the turret LEFT (toward positive angles). Negative turns it RIGHT.
     * The soft limits still stop it.
     */
    public void setManualPower(double power) {
        useBrake();
        manualPower = Range.clip(power, -1.0, 1.0) * MAX_MANUAL_POWER;
        wasWithinTolerance = false;
        onTarget = false;
        state = TurretState.MANUAL;
    }

    /*
     * Stop driving the turret. It floats, so it can be turned by hand. Safe to call every loop.
     */
    public void stop() {
        turretMotor.setPower(0.0);
        useFloat();
        manualPower = 0.0;
        wasWithinTolerance = false;
        onTarget = false;
        state = TurretState.IDLE;
    }

    /*
     * Call this ONCE per loop(), after you give the turret its commands.
     * It reads the encoder and the home switch, fixes the angle if the turret just reached
     * the switch, and then drives the motor.
     */
    public void update() {
        // 1. Read the encoder and work out the angle.
        encoderTicks = turretEncoder.getCurrentPosition();
        angleDegrees = ticksToAngleDegrees(encoderTicks);
        double secondsSinceLastLoop = loopTimer.seconds();
        loopTimer.reset();

        // 2. Check the home switch.
        boolean didRezero = updateHomeSwitch();

        // 3. How fast is the turret turning? (A re-zero makes the angle jump, so skip that loop.)
        if (didRezero || secondsSinceLastLoop <= 0.0) {
            angularVelocityDegreesPerSecond = 0.0;
        } else {
            angularVelocityDegreesPerSecond =
                    (angleDegrees - previousAngleDegrees) / secondsSinceLastLoop;
        }
        previousAngleDegrees = angleDegrees;

        // 4. Drive the motor. IDLE means we leave the motor alone.
        if (state == TurretState.MANUAL) {
            turretMotor.setPower(applySoftLimits(manualPower));
        } else if (state == TurretState.POSITION) {
            turretMotor.setPower(applySoftLimits(calculatePositionPower()));
        }

        // 5. Are we on target?
        updateOnTarget();
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks the turret what is happening
    // ==================================================================

    /*
     * True when the turret is in POSITION mode and has been within tolerance of its target
     * for a short time. This is the turret's part of the "OK to feed a ball" check.
     */
    public boolean isOnTarget() {
        return onTarget;
    }

    // True once the turret has been on the home switch at least once, so the angle is trusted.
    public boolean isZeroConfirmed() {
        return zeroConfirmed;
    }

    // True while the turret is sitting on the home switch (a magnet is near the sensor).
    public boolean isHomeSwitchActive() {
        return homeSwitch.getState() == HOME_SWITCH_ACTIVE_STATE;
    }

    public TurretState getState() {
        return state;
    }

    // The turret's angle in degrees as of the last update(). Positive is left.
    public double getAngleDegrees() {
        return angleDegrees;
    }

    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }

    // The raw encoder count. Use this to check the ticks per degree by hand.
    public int getEncoderTicks() {
        return encoderTicks;
    }

    /*
     * Adds the turret's own data to the telemetry. The SDK sends telemetry to the
     * Driver Station after every loop(), so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- TURRET ---");
        telemetry.addData("Turret State", state);
        telemetry.addData("Angle", "%.1f deg", angleDegrees);
        telemetry.addData("Target", "%.1f deg   (error %.1f)",
                targetAngleDegrees, targetAngleDegrees - angleDegrees);
        telemetry.addData("On Target?", onTarget);
        telemetry.addData("Zero Confirmed?", zeroConfirmed);
        telemetry.addData("Home Switch", "%s   (raw state = %b)",
                isHomeSwitchActive() ? "ON the switch" : "off the switch", homeSwitch.getState());
        telemetry.addData("Encoder ticks", encoderTicks);
        telemetry.addData("Soft limits", "%.0f to %.0f deg", MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
        telemetry.addData("Angle reached switch at", "%.1f deg  (before re-zero)", angleWhenReachedSwitch);
        telemetry.addData("Angle left switch at", "%.1f deg", angleWhenLeftSwitch);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    /*
     * Looks at the home switch. When the turret rolls onto the switch (and has re-armed),
     * the angle is reset to the home angle. Returns true if that just happened.
     */
    private boolean updateHomeSwitch() {
        boolean switchActive = isHomeSwitchActive();
        boolean justReachedSwitch = switchActive && !homeSwitchWasActive;
        boolean justLeftSwitch = !switchActive && homeSwitchWasActive;
        homeSwitchWasActive = switchActive;

        if (justLeftSwitch) {
            angleWhenLeftSwitch = angleDegrees;
        }

        // Once the turret has moved far enough from home, the next arrival counts again.
        if (!rearmed && Math.abs(angleDegrees - HOME_ANGLE_DEGREES) >= REARM_DISTANCE_DEGREES) {
            rearmed = true;
        }

        if (justReachedSwitch) {
            angleWhenReachedSwitch = angleDegrees;
            if (rearmed) {
                zeroAtHome();
                angleDegrees = HOME_ANGLE_DEGREES;
                return true;
            }
        }
        return false;
    }

    /*
     * Tells the encoder "this spot is the home angle". We reset its count to zero, and the
     * angle is worked out from the count from then on.
     * We use 0 ticks = HOME_ANGLE_DEGREES.
     */
    private void zeroAtHome() {
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        zeroConfirmed = true;
        rearmed = false;
    }

    /*
     * The position controller. Returns the motor power that moves the turret toward the target.
     */
    private double calculatePositionPower() {
        double error = targetAngleDegrees - angleDegrees;

        // Close enough: stop pushing and let the BRAKE hold the turret.
        if (Math.abs(error) <= ON_TARGET_TOLERANCE_DEGREES) {
            return 0.0;
        }

        // P pushes harder the further away we are. D slows the push when we are already
        // turning fast. Friction power gets the turret moving.
        double power = KP * error - KD * angularVelocityDegreesPerSecond;
        if (error > 0.0) {
            power = power + FRICTION_POWER;
        } else {
            power = power - FRICTION_POWER;
        }
        return Range.clip(power, -MAX_POWER, MAX_POWER);
    }

    /*
     * Never push the turret past a soft limit. At the limit, power that pushes further is
     * cut to zero. Power that pulls back toward the middle is still allowed.
     */
    private double applySoftLimits(double power) {
        if (angleDegrees >= MAX_ANGLE_DEGREES && power > 0.0) {
            return 0.0;
        }
        if (angleDegrees <= MIN_ANGLE_DEGREES && power < 0.0) {
            return 0.0;
        }
        return power;
    }

    /*
     * Works out whether we have been within tolerance of the target long enough.
     */
    private void updateOnTarget() {
        if (state != TurretState.POSITION) {
            wasWithinTolerance = false;
            onTarget = false;
            return;
        }

        boolean withinTolerance =
                Math.abs(targetAngleDegrees - angleDegrees) <= ON_TARGET_TOLERANCE_DEGREES;
        if (!withinTolerance) {
            wasWithinTolerance = false;
            onTarget = false;
            return;
        }

        // Within tolerance. Start the timer the first time, then wait out the hold time.
        if (!wasWithinTolerance) {
            onTargetTimer.reset();
            wasWithinTolerance = true;
        }
        if (onTargetTimer.seconds() >= ON_TARGET_HOLD_SECONDS) {
            onTarget = true;
        }
    }

    // Turns an encoder count into a turret angle. 0 ticks is the home angle.
    private double ticksToAngleDegrees(int ticks) {
        return HOME_ANGLE_DEGREES + ticks / TICKS_PER_TURRET_DEGREE;
    }

    // BRAKE holds the turret in place when power is 0. We only turn it on once the turret
    // is being commanded, so it is easy to turn by hand during INIT.
    private void useBrake() {
        if (brakeOn) {
            return;
        }
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brakeOn = true;
    }

    private void useFloat() {
        if (!brakeOn) {
            return;
        }
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brakeOn = false;
    }
}
