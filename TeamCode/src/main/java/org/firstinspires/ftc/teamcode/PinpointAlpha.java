package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

/*
 * Pinpoint Testing, Alpha
 * An iterative OpMode that checks the Pinpoint odometry on its own. The robot does NOT drive in
 * this OpMode. It owns NO motors. You PUSH THE ROBOT AROUND BY HAND and watch the numbers.
 * Everything it shows comes from PinpointSubsystem.
 *
 * CONTROLS (gamepad1)
 *   A -> set the robot's current spot as the new start: X = 0, Y = 0, heading = 0.
 *   B -> recalibrate the gyro AND zero the position. Hold the robot STILL when you press it.
 *
 * THE TEST ORDER (do these in order, and stop at the first one that fails)
 *   1. STATUS. After INIT, "Pinpoint status" must say READY. If it says NOT FOUND, the name in the
 *      robot configuration is wrong. If it says FAULT, check the cable it names.
 *   2. POD DIRECTIONS. Press A, then push the robot FORWARD by hand. X must go UP.
 *      Now push it LEFT (strafe). Y must go UP.
 *      If X or Y goes the wrong way, change that pod's direction in PinpointSubsystem
 *      (FORWARD_POD_DIRECTION or SIDEWAYS_POD_DIRECTION) from FORWARD to REVERSED, or back.
 *      Judge it by X and Y ONLY. The pod tick lines are RAW counts from the pod, before the
 *      direction setting is used, so they do not change when you reverse a pod (a pod that is
 *      set to REVERSED will show ticks going DOWN when it is correct). Use the ticks only to see
 *      that each pod is counting at all: the numbers must change when you push the robot.
 *   3. HEADING SIGN. Press B (robot still), then turn the robot LEFT by hand (counterclockwise,
 *      seen from above). "Heading" must go UP. If it goes DOWN, set INVERT_HEADING to true in
 *      PinpointSubsystem.
 *   4. DISTANCE. Press A. Mark a 48 inch line on the floor with a tape measure and push the robot
 *      straight along it, keeping the pods on the floor. X should show about 48. If it is off by
 *      a steady amount, put (real distance / shown distance) into DISTANCE_CALIBRATION_MULTIPLIER
 *      in PinpointSubsystem. Do the same sideways for Y.
 *   5. TURN IN PLACE (this checks the pod OFFSETS). Press A. Spin the robot a full 360 degrees
 *      LEFT, around the middle of the robot, until it points the same way it started.
 *      Heading should be back near 0, and X and Y should be back near 0 (within an inch or two).
 *      If X and Y wander when you spin, the pod offsets in PinpointSubsystem are wrong:
 *      remeasure FORWARD_POD_SIDEWAYS_OFFSET_INCHES and SIDEWAYS_POD_FORWARD_OFFSET_INCHES.
 *   6. DRIFT. Push the robot around for a minute, then bring it back to the start spot and the
 *      start heading. X, Y and heading should be back near 0. A few inches is normal.
 *
 * SAFE USE: nothing moves by itself here, so it is safe on the floor or on blocks. Keep the
 * spring-loaded pods pressing on the floor while you push, or the readings will be wrong.
 *
 * Robot configuration (on the Driver Station):
 *   I2C Device name: pinpoint   (PinpointSubsystem, "goBILDA Pinpoint Odometry Computer", Control Hub
 *                                I2C port 1, 2 or 3). See TeamCode/docs/pinpoint-setup-guide.md.
 */
@TeleOp(name = "Pinpoint - Alpha", group = "Testing")
public class PinpointAlpha extends OpMode {

    private PinpointSubsystem pinpointSubsystem;

    // Used so each button press counts ONCE, no matter how long it is held.
    private boolean wasAPressed = false;
    private boolean wasBPressed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The Pinpoint zeros itself and calibrates its gyro here, so keep the robot STILL.
        pinpointSubsystem = new PinpointSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Keep the robot still during INIT (the gyro is calibrating).");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Read it now, so you can check the status (test step 1) before pressing START.
        pinpointSubsystem.update();

        telemetry.addData("Status", "Ready to start!");
        addPinpointTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read the sensor.
        pinpointSubsystem.update();

        // 2. Read the gamepad and decide what to do. Only a NEW press counts.
        boolean aPressed = gamepad1.a;
        boolean bPressed = gamepad1.b;

        // 3. Tell the subsystem what to do.
        if (aPressed && !wasAPressed) {
            pinpointSubsystem.resetPose();
        }
        if (bPressed && !wasBPressed) {
            pinpointSubsystem.recalibrate();
        }
        wasAPressed = aPressed;
        wasBPressed = bPressed;

        // 4. Show what is happening on the Driver Station.
        telemetry.addLine("Push the robot by hand.  A = zero here.  B = recalibrate (robot STILL).");
        addPinpointTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Nothing to stop. The Pinpoint only reads, and this OpMode has no motors.
    }

    /*
     * The subsystem's own lines, plus the raw pod counts that the direction check needs.
     */
    private void addPinpointTelemetry() {
        pinpointSubsystem.addTelemetry(telemetry);
        if (pinpointSubsystem.isAvailable()) {
            // These are RAW counts (before the direction setting), so only check that they change.
            telemetry.addData("Forward pod ticks (raw: must change when pushed forward/back)",
                    pinpointSubsystem.getForwardPodTicks());
            telemetry.addData("Sideways pod ticks (raw: must change when pushed left/right)",
                    pinpointSubsystem.getSidewaysPodTicks());
            telemetry.addData("Pinpoint update rate", "%.0f Hz", pinpointSubsystem.getPinpointFrequencyHz());
        }
    }
}
