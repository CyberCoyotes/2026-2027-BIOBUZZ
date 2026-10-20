package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/*
 * Turret Testing
 * An iterative OpMode that tests the TurretSubsystem using gamepad2.
 *
 * CONTROLS (all on gamepad2)
 *   Left Stick X      -> MANUAL turning. Push the stick RIGHT and the turret turns RIGHT.
 *   D-pad Left        -> HOLD to turn the turret LEFT
 *   D-pad Right       -> HOLD to turn the turret RIGHT
 *   D-pad Up          -> turret POWER up by TURRET_POWER_PERCENT_STEP (one step per press)
 *   D-pad Down        -> turret POWER down by TURRET_POWER_PERCENT_STEP (one step per press)
 *   X (Square on PS5) -> PRESET: turn to FAR LEFT, +PRESET_SIDE_DEGREES (about +90)
 *   B (Circle on PS5) -> PRESET: turn to FAR RIGHT, -PRESET_SIDE_DEGREES (about -90)
 *   Y (Triangle)      -> PRESET: turn to CENTER, 0 degrees (0 encoder ticks, straight ahead)
 *   A (Cross on PS5)  -> STOP driving the turret. It floats so you can turn it by hand.
 *   Letting go of the stick or the D-pad Left / Right holds the turret where it is.
 *   A preset runs until the turret gets there, then holds. Press another button to change it.
 *   Priority: stick, then D-pad Left / Right, then Y, X, B, then A.
 *
 *   The presets use the ANGLE, so they only land in the right place if the angle is right.
 *   Do the "marked 90 degrees" check in the safe first test below BEFORE you use them.
 *   The presets use their own position-control power (MAX_POWER in TurretSubsystem),
 *   not the Turret power dial.
 *
 * TURRET POWER is how fast the turret turns. It starts at DEFAULT_TURRET_POWER_PERCENT (50%).
 * A full push on the stick, or holding D-pad Left / Right, turns the turret at this power.
 * A half push on the stick turns it at half of this power.
 *
 * ANGLES: left (counterclockwise) is POSITIVE, right is NEGATIVE, 0 is straight ahead.
 *
 * THE ANGLE LIMITS ARE OFF FOR NOW (SOFT_LIMITS_ON in TurretSubsystem is false). Nothing
 * stops the turret at an angle, so it can run into the cables or the frame. Keep your hand
 * near the Stop button, and start with a low power.
 *
 * HOME SWITCH IS OFF FOR NOW (not installed). 0 degrees is wherever the turret was when the
 * Control Hub powered on, so power on with the turret pointing straight ahead.
 *
 * SAFE FIRST TEST (do these in order, and keep your hand near the Stop button):
 *   1. INIT. Turn the turret LEFT by hand. The angle must go UP. If it goes down, flip
 *      INVERT_ENCODER_READING in TurretSubsystem. (Do NOT flip TURRET_MOTOR_DIRECTION for this.
 *      It would also reverse the motor and the encoder together, and change nothing here.)
 *   2. Check ticks per degree: start with the turret straight ahead, then turn it by hand
 *      through a marked 90 degrees. The angle should read about 90. If not, check the tooth
 *      counts in TurretSubsystem.
 *   3. START. Push the stick a LITTLE to the right. The turret must turn RIGHT and the angle
 *      must go DOWN. If the turret turns the wrong way, flip TURRET_MOTOR_DIRECTION.
 *   4. Hold D-pad Left and Right. The turret must turn only while held, and stop when you let
 *      go. Press D-pad Up and Down and watch the Turret power line change.
 *   5. Try the presets from the center: Y should stay put, X should swing LEFT and B should
 *      swing RIGHT, each stopping at about 90 degrees on the screen. Press A to stop it.
 *   6. When all of that works, set SOFT_LIMITS_ON to true in TurretSubsystem and set the
 *      limits to about +/-130 degrees.
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name:           turret_motor        (goBILDA 5203 motor; its encoder cable is plugged
 *                                              into the same port's encoder connector)
 *   (Home switch, Digital Device "turret_home_sensor": not installed yet, so not needed.)
 */
@TeleOp(name = "Turret Testing", group = "Testing")
public class TurretAlpha extends OpMode {

    // The turret power dial, preset angles and stick deadband are shared with BiobuzzTeleOp,
    // so they live in OpModeConstants.java. Change them there, once.

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TurretSubsystem turretSubsystem;

    // The turret power right now, in percent. D-pad Up and Down change it.
    private int turretPowerPercent = OpModeConstants.TURRET_POWER_PERCENT_START;

    // Remember the D-pad Up and Down buttons from last loop. We only want ONE step per press,
    // not a step every loop while the button is held down.
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;

    // Remembers whether the stick or the D-pad was turning the turret last loop, so we can
    // hold the angle when it is released.
    private boolean wasTurningManually = false;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS (gamepad2) ---");
        telemetry.addLine("Left Stick X = MANUAL (right = turn right)");
        telemetry.addLine("D-pad Left / Right = HOLD to turn (let go to stop)");
        telemetry.addLine(String.format("D-pad Up / Down = turret power +/- %d%%", OpModeConstants.TURRET_POWER_PERCENT_STEP));
        telemetry.addLine(String.format("Y = center (%.0f)   X = far left (%+.0f)   B = far right (%+.0f) degrees",
                OpModeConstants.TURRET_PRESET_CENTER_DEGREES, OpModeConstants.TURRET_PRESET_LEFT_DEGREES, OpModeConstants.TURRET_PRESET_RIGHT_DEGREES));
        telemetry.addLine("A = STOP (turret floats)");
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The subsystem gets its own hardware from the hardware map.
        // If the turret is sitting on the home switch, it zeroes itself right here.
        turretSubsystem = new TurretSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        addControlsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // The turret is not driven during INIT. We only read it, so you can turn it by hand.
        turretSubsystem.update();

        // HOME SWITCH OFF: the switch is not installed, so this check would always say
        // "NOT AT HOME". Put it back when the switch is installed.
        telemetry.addData("Status", "Home switch not installed. Turret 0 deg = where it was at power-up.");
        // Pre-match check: the turret must start ON the home switch.
        // if (turretSubsystem.isHomeSwitchActive()) {
        //     telemetry.addData("Status", "Ready to start! Turret is on the home switch.");
        // } else {
        //     telemetry.addData("Status", "TURRET NOT AT HOME! Turn it by hand onto the switch.");
        // }
        addControlsTelemetry();
        turretSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors. (The turret reads its own encoder inside update(), below.)

        // 2. Read the gamepad and decide what to do. The first one that is true wins.
        String turretControl;

        // D-pad Up and Down change the turret power. Only a NEW press counts, so holding the
        // button down gives one step, not a step every loop.
        boolean dpadUpPressed = gamepad2.dpad_up;
        boolean dpadDownPressed = gamepad2.dpad_down;
        if (dpadUpPressed && !wasDpadUpPressed) {
            turretPowerPercent += OpModeConstants.TURRET_POWER_PERCENT_STEP;
        }
        if (dpadDownPressed && !wasDpadDownPressed) {
            turretPowerPercent -= OpModeConstants.TURRET_POWER_PERCENT_STEP;
        }
        turretPowerPercent = Math.max(OpModeConstants.TURRET_POWER_PERCENT_MIN,
                Math.min(OpModeConstants.TURRET_POWER_PERCENT_MAX, turretPowerPercent));
        wasDpadUpPressed = dpadUpPressed;
        wasDpadDownPressed = dpadDownPressed;

        // Turn the percent into a number from 0.0 to 1.0.
        double turretPower = turretPowerPercent / 100.0;

        // Stick RIGHT is a positive number on the controller. But turning RIGHT means going
        // clockwise, and clockwise is a NEGATIVE angle. So we flip the sign of the stick.
        double stick = -gamepad2.left_stick_x;
        boolean usingStick = Math.abs(stick) > OpModeConstants.TURRET_STICK_DEADBAND;

        // The D-pad turns the turret only while it is held. Pressing Left and Right together
        // cancels out, so neither counts.
        boolean dpadLeftOnly = gamepad2.dpad_left && !gamepad2.dpad_right;
        boolean dpadRightOnly = gamepad2.dpad_right && !gamepad2.dpad_left;
        boolean turningManually = usingStick || dpadLeftOnly || dpadRightOnly;

        // 3. Tell the turret what to do, then call update().
        if (usingStick) {
            // The stick scales the power: a full push is the whole turret power,
            // a half push is half of it.
            turretSubsystem.setManualPower(stick * turretPower);
            turretControl = "Left Stick (MANUAL)";
        } else if (dpadLeftOnly) {
            // Left is a POSITIVE angle, so positive power.
            turretSubsystem.setManualPower(turretPower);
            turretControl = "D-pad Left (MANUAL)";
        } else if (dpadRightOnly) {
            turretSubsystem.setManualPower(-turretPower);
            turretControl = "D-pad Right (MANUAL)";
        } else if (wasTurningManually) {
            // The stick or D-pad was just let go, so hold the turret where it stopped.
            turretSubsystem.holdCurrentAngle();
            turretControl = "Released (HOLD)";
        } else if (gamepad2.y) {
            turretSubsystem.setTargetAngle(OpModeConstants.TURRET_PRESET_CENTER_DEGREES);
            turretControl = "Y (preset CENTER)";
        } else if (gamepad2.x) {
            turretSubsystem.setTargetAngle(OpModeConstants.TURRET_PRESET_LEFT_DEGREES);
            turretControl = "X (preset FAR LEFT)";
        } else if (gamepad2.b) {
            turretSubsystem.setTargetAngle(OpModeConstants.TURRET_PRESET_RIGHT_DEGREES);
            turretControl = "B (preset FAR RIGHT)";
        } else if (gamepad2.a) {
            turretSubsystem.stop();
            turretControl = "A (STOP)";
        } else {
            // No new command. The turret keeps doing what it was last told to do.
            turretControl = "None (keeps last command)";
        }
        wasTurningManually = turningManually;
        turretSubsystem.update();

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Control", turretControl);
        telemetry.addData("Turret power", "%d%%", turretPowerPercent);
        turretSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Always leave the motor stopped.
        turretSubsystem.stop();
    }
}
