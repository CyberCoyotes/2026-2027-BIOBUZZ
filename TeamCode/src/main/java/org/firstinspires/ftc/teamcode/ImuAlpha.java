package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.BiobuzzLog;
import org.firstinspires.ftc.teamcode.subsystems.HiveBearingMemory;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerMonitorSubsystem;

/*
 * IMU Testing, Alpha
 * An iterative OpMode that checks the Control Hub's built-in gyro (the IMU) on its own, and
 * compares it with the Pinpoint's heading. The robot does NOT drive in this OpMode. It owns NO
 * motors. You TURN THE ROBOT BY HAND and watch the numbers.
 * Everything it shows comes from ImuSubsystem, PinpointSubsystem and PowerMonitorSubsystem.
 *
 * TWO JOBS
 *   1. Check the hub's orientation settings (ImuSubsystem: LOGO_FACING_DIRECTION and
 *      USB_FACING_DIRECTION) and the heading's sign. Do this ONCE, before trusting field-centric.
 *   2. Be a witness. The IMU and the Pinpoint both measure which way the robot faces. If the
 *      Pinpoint blinks or restarts, ITS heading jumps while the IMU's does not, and this OpMode
 *      writes it down (on the screen and in the Robot Controller log, tag BIOBUZZ) with the
 *      Pinpoint's status and the battery voltage. Leave it running while you drive the robot
 *      around by hand, or run it on the robot during practice, then read the log:
 *           adb logcat -d | grep BIOBUZZ
 *
 * CONTROLS (gamepad1)
 *   A -> make the way the robot faces now the zero for BOTH the IMU and the Pinpoint.
 *
 * THE TEST ORDER (do these in order, and stop at the first one that fails)
 *   1. STATUS. After INIT, "IMU heading" must show a number. If it says NOT FOUND, the config has
 *      no IMU named "imu" (see ImuSubsystem). If it says it did NOT start up, the orientation
 *      settings are impossible (the logo and USB directions cannot be on the same axis).
 *   2. FLAT. Put the robot on the floor, flat, and press A. "pitch, roll" must both be about 0.
 *      If one of them reads about 90, or they are large, the hub's orientation is wrong: look at the
 *      hub, and fix LOGO_FACING_DIRECTION and USB_FACING_DIRECTION in ImuSubsystem. (Our hub stands
 *      on its end on the robot's LEFT wall, with its USB ports at the ceiling and its logo
 *      facing the middle of the robot: so USB is UP and the logo is RIGHT. If this step fails, the
 *      logo word in ImuSubsystem is the first thing to check: if "left wall" meant the viewer's
 *      left and not the robot's, it should be LEFT.)
 *   3. TIPPING. Lift the FRONT of the robot a little and put it back. PITCH must change and ROLL
 *      must stay near 0. Lift the LEFT side a little: ROLL must change and PITCH stay near 0. If it
 *      is the other way round, the orientation settings are wrong (fix them, then repeat 2 and 3).
 *   4. HEADING SIGN. Press A, then turn the robot LEFT by hand (counterclockwise, seen from above)
 *      about a quarter turn. "IMU heading" must go UP, to about +90. If it goes DOWN while steps 2
 *      and 3 passed, set INVERT_HEADING to true in ImuSubsystem. If it goes down AND steps 2 or 3
 *      failed, fix the orientation first.
 *   5. FULL TURN. Press A and spin the robot a full 360 degrees LEFT, then put it back facing the
 *      way it started. The IMU heading should be back near 0 (within a couple of degrees).
 *   6. COMPARE. The "Pinpoint minus IMU" line shows how far apart the two headings are. Turn the
 *      robot around by hand for a minute. It should stay within a few degrees and drift slowly.
 *      "Headings disagreed" counts every moment one moved and the other did not (more than
 *      DISAGREE_JUMP_DEGREES in one loop). That should stay 0. Anything above 0 means one of the two
 *      blinked: the Pinpoint's own drop log lines and the log tell you which one and why.
 *
 * SAFE USE: nothing moves by itself here, so it is safe on the floor or on blocks.
 *
 * Robot configuration (on the Driver Station):
 *   IMU name: imu           (ImuSubsystem, the Control Hub's built-in IMU, I2C Bus 0)
 *   I2C Device name: pinpoint   (PinpointSubsystem; if it is missing, only the IMU half works)
 */
@TeleOp(name = "IMU - Alpha", group = "Testing")
public class ImuAlpha extends OpMode {

    // ------------------------------------------------------------------
    // Test-only numbers
    // ------------------------------------------------------------------
    // The gap between the Pinpoint's heading and the IMU's heading is allowed to change by this
    // many degrees from one loop to the next. A real turn changes BOTH headings together, so the
    // gap barely moves. A bigger change means ONE of them jumped on its own. (The two are read a
    // few milliseconds apart, so a fast spin makes the gap wobble by a degree or two. 10 is far
    // above that.) Smaller = catches smaller jumps but may cry wolf. Bigger = the opposite.
    private static final double DISAGREE_JUMP_DEGREES = 10.0;

    private ImuSubsystem imuSubsystem;
    private PinpointSubsystem pinpointSubsystem;
    private PowerMonitorSubsystem powerMonitorSubsystem;

    // Used so each button press counts ONCE, no matter how long it is held.
    private boolean wasAPressed = false;

    // The "Pinpoint minus IMU" gap from the last loop, so we can spot a jump in it.
    private double lastGapDegrees = 0.0;
    private boolean haveLastGap = false;

    // For the screen and the log.
    private int disagreementCount = 0;
    private double biggestGapDegrees = 0.0;
    private String lastDisagreementText = "none";
    private final ElapsedTime runTimer = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The IMU takes the way the robot faces now as zero. (It does not need the robot to be
        // still, but the Pinpoint does: it calibrates its gyro here.)
        imuSubsystem = new ImuSubsystem(hardwareMap);
        pinpointSubsystem = new PinpointSubsystem(hardwareMap);
        powerMonitorSubsystem = new PowerMonitorSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Keep the robot still during INIT (the Pinpoint's gyro is calibrating).");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Read everything now, so you can check the status (test step 1) before pressing START.
        imuSubsystem.update();
        pinpointSubsystem.update();
        powerMonitorSubsystem.update();

        telemetry.addData("Status", "Ready to start!");
        imuSubsystem.addTelemetry(telemetry);
        pinpointSubsystem.addTelemetry(telemetry);
        powerMonitorSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runTimer.reset();
        powerMonitorSubsystem.resetStatistics();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read the sensors.
        imuSubsystem.update();
        pinpointSubsystem.update();
        powerMonitorSubsystem.update();

        // 2. Read the gamepad and decide what to do. Only a NEW press counts.
        boolean aPressed = gamepad1.a;

        // 3. Tell the subsystems what to do.
        if (aPressed && !wasAPressed) {
            imuSubsystem.resetHeading();
            pinpointSubsystem.resetPose();
            haveLastGap = false;
            biggestGapDegrees = 0.0;
        }
        wasAPressed = aPressed;

        checkHeadingsAgree();

        // 4. Show what is happening on the Driver Station.
        telemetry.addLine("Turn the robot by hand.  A = zero both headings here.");
        imuSubsystem.addTelemetry(telemetry);
        addComparisonTelemetry();
        pinpointSubsystem.addTelemetry(telemetry);
        powerMonitorSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Nothing to stop. These classes only read, and this OpMode has no motors.
    }

    /*
     * Watches the gap between the Pinpoint's heading and the IMU's heading. A real turn moves both
     * together, so the gap stays put. If the gap suddenly jumps, one of the two blinked, so write
     * it down with everything we know. Only works while both are available and the Pinpoint READY.
     */
    private void checkHeadingsAgree() {
        if (!imuSubsystem.isAvailable() || !pinpointSubsystem.isReady()) {
            // We cannot compare right now. Forget the old gap, so a comparison that starts again
            // later does not look like a jump.
            haveLastGap = false;
            return;
        }

        double gap = HiveBearingMemory.wrapDegrees(
                pinpointSubsystem.getHeadingDegrees() - imuSubsystem.getHeadingDegrees());
        if (Math.abs(gap) > biggestGapDegrees) {
            biggestGapDegrees = Math.abs(gap);
        }

        if (haveLastGap) {
            double jump = HiveBearingMemory.wrapDegrees(gap - lastGapDegrees);
            if (Math.abs(jump) > DISAGREE_JUMP_DEGREES) {
                disagreementCount = disagreementCount + 1;
                lastDisagreementText = String.format("at %.1f s: the gap jumped %+.0f deg (Pinpoint %.1f, IMU %.1f); %s",
                        runTimer.seconds(), jump, pinpointSubsystem.getHeadingDegrees(),
                        imuSubsystem.getHeadingDegrees(), powerMonitorSubsystem.getSummaryText());
                BiobuzzLog.warning("HEADINGS DISAGREED #%d %s", disagreementCount, lastDisagreementText);
            }
        }
        lastGapDegrees = gap;
        haveLastGap = true;
    }

    /*
     * The IMU-versus-Pinpoint lines for the screen.
     */
    private void addComparisonTelemetry() {
        telemetry.addLine("--- IMU versus PINPOINT (test step 6) ---");
        if (!imuSubsystem.isAvailable() || !pinpointSubsystem.isAvailable()) {
            telemetry.addData("Compare", "needs BOTH the IMU and the Pinpoint in the robot config");
            return;
        }
        if (!pinpointSubsystem.isReady()) {
            telemetry.addData("Compare", "waiting for the Pinpoint to say READY (it is %s)", pinpointSubsystem.getDeviceStatus());
            return;
        }
        telemetry.addData("Pinpoint minus IMU", "%+.1f deg   (biggest so far %.1f deg)", lastGapDegrees, biggestGapDegrees);
        telemetry.addData("Headings disagreed", "%d time(s)   (a jump of more than %.0f deg in one loop)",
                disagreementCount, DISAGREE_JUMP_DEGREES);
        telemetry.addData("Last disagreement", lastDisagreementText);
    }
}
