package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/*
 * Turret Testing
 * An iterative OpMode that tests the TurretSubsystem using gamepad1.
 *
 * CONTROLS
 *   Left Stick X      -> MANUAL turning. Push the stick RIGHT and the turret turns RIGHT.
 *   D-pad Up          -> turn to 0 degrees (straight ahead, the intake direction)
 *   D-pad Left        -> turn to +PRESET_SIDE_DEGREES (left)
 *   D-pad Right       -> turn to -PRESET_SIDE_DEGREES (right)
 *   B (Circle on PS5) -> STOP driving the turret. It floats so you can turn it by hand.
 *   Letting go of the stick holds the turret where it is.
 *   Priority: stick, then D-pad, then B.
 *
 * ANGLES: left (counterclockwise) is POSITIVE, right is NEGATIVE, 0 is straight ahead.
 * The soft limits in TurretSubsystem start small (+/-45 degrees), so a D-pad preset that
 * is further than that is held to the limit. The Target line shows what it really used.
 *
 * SAFE FIRST TEST (do these in order, and keep your hand near the Stop button):
 *   1. INIT with the turret placed by hand ON the home switch. Read the Home Switch line.
 *      Move the turret off the switch by hand. The line must change. If it looks backwards,
 *      flip HOME_SWITCH_ACTIVE_STATE in TurretSubsystem.
 *   2. Turn the turret LEFT by hand. The angle must go UP. If it goes down, flip
 *      TURRET_ENCODER_DIRECTION in TurretSubsystem.
 *   3. Check ticks per degree: put the turret on the switch, then turn it by hand through a
 *      marked 90 degrees. The angle should read about 90. If not, check the tooth counts.
 *   4. START. Push the stick a LITTLE to the right. The turret must turn RIGHT and the angle
 *      must go DOWN. If the turret turns the wrong way, flip TURRET_MOTOR_DIRECTION.
 *   5. Try the D-pad. Then roll the turret across the switch and watch "Angle reached switch at".
 *      That number is how far the angle had drifted before it was re-zeroed.
 *   6. When all of that works, widen the soft limits in TurretSubsystem to about +/-130
 *      and raise PRESET_SIDE_DEGREES below to 90.
 *
 * On a PS5 controller: A = Cross, B = Circle, X = Square, Y = Triangle.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name:           turret_motor        (goBILDA motor)
 *   Motor name:           turret_encoder      (REV Through Bore Encoder, no motor attached)
 *   Digital Device name:  turret_home_switch  (REV magnetic limit switch)
 */
@TeleOp(name = "Turret Testing", group = "Testing")
public class TurretTesting extends OpMode {

    // The D-pad Left and Right presets turn this many degrees to each side.
    // Start small. Raise it to 90 after the soft limits are widened.
    private static final double PRESET_SIDE_DEGREES = 30.0;

    // The straight-ahead preset (the intake direction).
    private static final double PRESET_FORWARD_DEGREES = 0.0;

    // The stick must move more than this before it counts, so a stick that does not
    // quite rest at zero will not turn the turret.
    private static final double STICK_DEADBAND = 0.1;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TurretSubsystem turretSubsystem;

    // Remembers whether the stick was in use last loop, so we can hold the angle when it is released.
    private boolean wasUsingStick = false;

    /*
     * Adds the button map to telemetry so the drivers can read it.
     */
    private void addControlsTelemetry() {
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Left Stick X = MANUAL (right = turn right)");
        telemetry.addLine(String.format("D-pad Up = %.0f   Left = +%.0f   Right = -%.0f degrees",
                PRESET_FORWARD_DEGREES, PRESET_SIDE_DEGREES, PRESET_SIDE_DEGREES));
        telemetry.addLine("B = STOP (turret floats)");
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

        // Pre-match check: the turret must start ON the home switch.
        if (turretSubsystem.isHomeSwitchActive()) {
            telemetry.addData("Status", "Ready to start! Turret is on the home switch.");
        } else {
            telemetry.addData("Status", "TURRET NOT AT HOME! Turn it by hand onto the switch.");
        }
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
        // 1. Read sensors. (The turret reads its own encoder and switch inside update(), below.)

        // 2. Read the gamepad and decide what to do. The first one that is true wins.
        String turretControl;

        // Stick RIGHT is a positive number on the controller. But turning RIGHT means going
        // clockwise, and clockwise is a NEGATIVE angle. So we flip the sign of the stick.
        double manualPower = -gamepad1.left_stick_x;
        boolean usingStick = Math.abs(manualPower) > STICK_DEADBAND;

        // 3. Tell the turret what to do, then call update().
        if (usingStick) {
            turretSubsystem.setManualPower(manualPower);
            turretControl = "Left Stick (MANUAL)";
        } else if (wasUsingStick) {
            // The stick was just let go, so hold the turret where it stopped.
            turretSubsystem.holdCurrentAngle();
            turretControl = "Stick released (HOLD)";
        } else if (gamepad1.dpad_up) {
            turretSubsystem.setTargetAngle(PRESET_FORWARD_DEGREES);
            turretControl = "D-pad Up";
        } else if (gamepad1.dpad_left) {
            turretSubsystem.setTargetAngle(PRESET_SIDE_DEGREES);
            turretControl = "D-pad Left";
        } else if (gamepad1.dpad_right) {
            turretSubsystem.setTargetAngle(-PRESET_SIDE_DEGREES);
            turretControl = "D-pad Right";
        } else if (gamepad1.b) {
            turretSubsystem.stop();
            turretControl = "B (STOP)";
        } else {
            // No new command. The turret keeps doing what it was last told to do.
            turretControl = "None (keeps last command)";
        }
        wasUsingStick = usingStick;
        turretSubsystem.update();

        // 4. Show what is happening on the Driver Station.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Control", turretControl);
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
