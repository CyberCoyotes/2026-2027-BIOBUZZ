package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Intake Subsystem
 * Owns the intake motor. Only this class touches it.
 * OpModes tell it what to do with intake(), reverse() and stop().
 *
 * WHAT THE MOTOR DOES
 *   ONE motor (goBILDA 5203 series, 1620 RPM, which is the 3.7:1 gearbox) does two jobs:
 *     1. It spins the front intake that pulls balls in.
 *     2. It also powers a second hex shaft with grippers in the transfer chute.
 *   So when the intake runs, the transfer grippers run too. Both turn together, and we cannot
 *   run one without the other.
 *
 * POWER MODE, NOT RPM
 *   We control the motor with plain power (-1.0 to 1.0), not a target speed. An intake only has
 *   to grab balls and push them in. It does not need an exact speed the way the flywheel does.
 *   If we ever want a steady speed as the battery drains, the speed control would go INSIDE this
 *   class and the OpModes would not change.
 *
 * ONE POWER DIAL
 *   There is ONE power number (setIntakePower). intake() runs the motor at +that number, and
 *   reverse() runs it at -that number. Easy to split into two numbers later if we want to.
 *
 * STATE (what we told it to do) is IntakeState and the power dial.
 * STATUS (what the sensor reports) is the encoder speed in ticks per second.
 *   The encoder is plugged in for testing only. We do NOT turn ticks into RPM, because we have
 *   not yet confirmed if the motor counts 28 or 112 ticks per revolution (see CLAUDE.md).
 *   If the encoder is not plugged in, the speed just reads 0 and nothing else is affected.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: intake_motor
 *   Motor type: goBILDA 5202/3/4 series
 */
public class IntakeSubsystem {

    // ------------------------------------------------------------------
    // Hardware name. This MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    private static final String INTAKE_MOTOR_NAME = "intake_motor";

    // ------------------------------------------------------------------
    // Motor settings. TEST THESE on the robot!
    // ------------------------------------------------------------------
    // If intake() pushes balls OUT instead of IN, flip this between FORWARD and REVERSE.
    // It starts as REVERSE because that is what our old IntakeAlpha used.
    private static final DcMotor.Direction INTAKE_DIRECTION = DcMotor.Direction.REVERSE;

    // FLOAT lets the intake coast to a stop. Try BRAKE if you want it to stop instantly.
    private static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.FLOAT;

    // ------------------------------------------------------------------
    // Power settings (0.0 to 1.0)
    // ------------------------------------------------------------------
    // The power the dial starts at. Start gentle (50%) on a new mechanism.
    // Once IntakeAlpha shows us the best number, change it here.
    private static final double DEFAULT_INTAKE_POWER = 0.50;

    // The most the power dial can be turned up to. 1.0 is full power.
    private static final double MAX_INTAKE_POWER = 1.0;

    // What the intake is doing right now.
    public enum IntakeState {
        STOPPED,    // the motor is off
        INTAKING,   // pulling balls in (positive power)
        REVERSING   // pushing balls back out (negative power)
    }

    // ------------------------------------------------------------------
    // Hardware and data. Only this class changes these.
    // ------------------------------------------------------------------
    // DcMotorEx is a DcMotor with extra features. We only use it to READ the speed.
    private DcMotorEx intakeMotor;

    // STATE: what we told it to do.
    private IntakeState intakeState = IntakeState.STOPPED;
    private double intakePower = DEFAULT_INTAKE_POWER;

    // The power we last sent to the motor (negative means reverse).
    private double motorPower = 0.0;

    // STATUS: what the encoder reports.
    private double ticksPerSecond = 0.0;

    /*
     * Setup. Gets the motor from the robot configuration and sets it up.
     * The intake starts stopped.
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        // The name MUST match the robot configuration exactly.
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);

        intakeMotor.setDirection(INTAKE_DIRECTION);

        // We control power directly, not speed. RUN_WITHOUT_ENCODER still lets us READ the
        // encoder, we just do not use it to control the motor.
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

        stop();
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the intake what to do
    // ==================================================================

    /*
     * Pull balls IN, at the power dial setting. Safe to call every loop.
     */
    public void intake() {
        intakeState = IntakeState.INTAKING;
        setMotorPower(intakePower);
    }

    /*
     * Push balls back OUT (to clear a jam), at the power dial setting. Safe to call every loop.
     */
    public void reverse() {
        intakeState = IntakeState.REVERSING;
        setMotorPower(-intakePower);
    }

    /*
     * Stop the intake. Safe to call every loop.
     */
    public void stop() {
        intakeState = IntakeState.STOPPED;
        setMotorPower(0.0);
    }

    /*
     * Turn the power dial. 0.0 to MAX_INTAKE_POWER, where 1.0 is full power.
     * It stays set until you change it. It takes effect the next time intake() or reverse() is
     * called, so call one of them every loop (IntakeAlpha does).
     */
    public void setIntakePower(double power) {
        intakePower = Range.clip(power, 0.0, MAX_INTAKE_POWER);
    }

    /*
     * Reads the encoder. Call this once per loop(), BEFORE asking for the speed.
     */
    public void update() {
        ticksPerSecond = intakeMotor.getVelocity();
    }

    /*
     * Sends a power to the motor and remembers it for telemetry.
     */
    private void setMotorPower(double power) {
        motorPower = Range.clip(power, -1.0, 1.0);
        intakeMotor.setPower(motorPower);
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks the intake what is happening
    // ==================================================================

    public IntakeState getState() {
        return intakeState;
    }

    // The power dial setting (0.0 to 1.0). Always positive.
    public double getIntakePower() {
        return intakePower;
    }

    // The power last sent to the motor (negative when reversing).
    public double getMotorPower() {
        return motorPower;
    }

    // The encoder speed, in ticks per second, as of the last update().
    public double getTicksPerSecond() {
        return ticksPerSecond;
    }

    /*
     * Adds the intake's own data to the telemetry. The SDK sends telemetry to the
     * Driver Station after every loop(), so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- INTAKE ---");
        telemetry.addData("Intake state", intakeState);
        telemetry.addData("Power dial", "%.0f%%", intakePower * 100);
        telemetry.addData("Motor power", "%.0f%%", motorPower * 100);
        telemetry.addData("Encoder speed", "%.0f ticks/sec", ticksPerSecond);
    }
}
