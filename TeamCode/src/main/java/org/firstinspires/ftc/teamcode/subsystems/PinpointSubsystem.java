package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/*
 * Pinpoint Subsystem
 * Owns the goBILDA PINPOINT odometry computer and its two odometry pods. It only READS. It never
 * moves a motor, and it has no gamepad code. Another class (for now TeleOp - Alpha 8 and
 * PinpointAlpha) asks it where the robot is.
 *
 * WHAT IT ANSWERS: "Where is the robot, and which way is it facing, compared to where it started?"
 *   getXInches()          -> how far FORWARD of the start point. Forward is POSITIVE.
 *   getYInches()          -> how far LEFT of the start point. Left is POSITIVE.
 *   getHeadingDegrees()   -> which way the robot faces. Turning LEFT (counterclockwise) is POSITIVE.
 * Those are the same signs as DriveSubsystem and TurretSubsystem (forward +, left +, counter-
 * clockwise +), and the same as Pedro Pathing. So the turret's field direction is later just:
 *   shooterFieldHeading = robotHeading + turretAngle
 *
 * HOW IT WORKS: two spring-loaded (swingarm) odometry pods ride on the floor. One pod faces
 * forward (it counts when the robot drives forward or backward) and one faces sideways (it counts
 * when the robot strafes). The Pinpoint adds in its own gyro (the IMU) to work out the turning.
 * The pods are not wheels that can slip, so this is more accurate than counting drive wheel turns.
 * It still drifts a little over a whole match, so treat it as "close", not "exact".
 *
 * STATE vs STATUS: this class gives the robot no commands, so there is no state. The STATUS is
 * what the Pinpoint reports about itself (getDeviceStatus()): READY means it is working.
 *
 * ABSENT DEVICE: if the robot's configuration has no device named PINPOINT_NAME, this class does
 * NOT stop the OpMode. Every getter answers 0, isAvailable() is false, and the telemetry says
 * what to fix. (The robot can still drive. It just does not know where it is.)
 *
 * The OpMode must call update() ONCE per loop(). Nothing calls it for us.
 *
 * Robot configuration (on the Driver Station):
 *   I2C Device name: pinpoint   (type "goBILDA Pinpoint Odometry Computer", plugged into one of the
 *                                Control Hub's I2C ports 1, 2 or 3. Port 0 is the Hub's own IMU.)
 *   Last season this was named "odo". It has to be renamed once on each robot. The full steps are
 *   in TeamCode/docs/pinpoint-setup-guide.md.
 *
 * NOTE FOR AUTO: in Autonomous, Pedro Pathing reads the Pinpoint by itself (its localizer). Never
 * use this class and a Pedro Follower in the same OpMode, because both would talk to the device.
 */
public class PinpointSubsystem {

    // ------------------------------------------------------------------
    // Hardware name. This MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    private static final String PINPOINT_NAME = "pinpoint";

    // ------------------------------------------------------------------
    // Where the pods are. These numbers are copied from last season's code (same chassis), and
    // were measured then. [CONFIRM] Measure them again with a tape measure if either pod has
    // been moved. Wrong numbers do NOT hurt the heading, but the X and Y readings drift sideways
    // whenever the robot turns.
    // ------------------------------------------------------------------
    // Measured from the robot's center of turning, to the middle of each pod's wheel.
    //
    // The FORWARD pod (it counts forward and backward) sits to one side of the center.
    //   LEFT of center is POSITIVE, RIGHT of center is NEGATIVE.
    //   Ours is 3.75 inches to the RIGHT, so it is -3.75.
    private static final double FORWARD_POD_SIDEWAYS_OFFSET_INCHES = -3.75;

    // The SIDEWAYS pod (it counts left and right) sits ahead of or behind the center.
    //   FORWARD of center is POSITIVE, BEHIND center is NEGATIVE.
    //   Ours is centered front to back, so it is 0.0.
    private static final double SIDEWAYS_POD_FORWARD_OFFSET_INCHES = 0.0;

    // ------------------------------------------------------------------
    // The pods themselves
    // ------------------------------------------------------------------
    // goBILDA makes two kinds of odometry pod, and each counts a different number of ticks per
    // millimeter. Ours are the spring-loaded SWINGARM pods. (The other kind is goBILDA_4_BAR_POD.)
    // If the readings are off by a steady amount, check this first.
    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;

    // Which way each pod counts. The rule is: X must go UP when the robot is pushed FORWARD, and
    // Y must go UP when the robot is pushed LEFT. If either one goes the wrong way, change that
    // one to REVERSED. (Last season both were FORWARD.)
    // TESTED on the robot: the forward pod was right as FORWARD. The sideways pod counted
    // NEGATIVE when the robot was pushed LEFT, so it is REVERSED. (The pod is mounted with its
    // thick side toward the front, which is the opposite of last season. It does not need to be
    // flipped by hand, because this setting does the same job.)
    private static final GoBildaPinpointDriver.EncoderDirection FORWARD_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection SIDEWAYS_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // ------------------------------------------------------------------
    // Corrections found by testing on the robot
    // ------------------------------------------------------------------
    // Heading must go UP when the robot turns LEFT (counterclockwise). The Pinpoint can be
    // mounted right side up or upside down, and that can flip the sign. If turning the robot LEFT
    // makes the heading go DOWN, change this to true.
    // TESTED on the robot: turning RIGHT gave a negative heading and turning LEFT a positive one,
    // so false is right. (Last season's helper flipped it, but the Pedro Pathing setup did not.)
    private static final boolean INVERT_HEADING = false;

    // Distance fix. Push the robot a measured distance (say 48 inches) and see what X says.
    //   multiplier = real distance / distance the screen shows
    // Start at 1.0. Last season's code used 1.5 (it showed 6 inches for a real 9). We did NOT copy
    // that, because 1.5 is exactly the ratio between the two pod settings (19.894 / 13.263), so it
    // probably covered up a pod setting that was not taking effect. Measure first, and if it is
    // still off by a steady amount, put the number here. It is applied to X, Y and their speeds.
    private static final double DISTANCE_CALIBRATION_MULTIPLIER = 1.0;

    // ------------------------------------------------------------------
    // The hardware
    // ------------------------------------------------------------------
    // null when the robot's configuration has no device with PINPOINT_NAME.
    private final GoBildaPinpointDriver pinpoint;

    // The newest answers, saved by update(). They are zero when there is no Pinpoint.
    private double xInches = 0.0;
    private double yInches = 0.0;
    private double headingDegrees = 0.0;
    private double xSpeedInchesPerSecond = 0.0;
    private double ySpeedInchesPerSecond = 0.0;
    private double headingSpeedDegreesPerSecond = 0.0;

    // ==================================================================
    // SETUP
    // ==================================================================

    /*
     * Gets the Pinpoint, tells it where the pods are, and zeros the position and the gyro.
     * The robot must be STILL when this runs (at INIT), because the gyro calibrates now.
     */
    public PinpointSubsystem(HardwareMap hardwareMap) {
        // tryGet gives back null instead of stopping the OpMode when the name is not in the
        // robot configuration.
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT_NAME);
        if (pinpoint == null) {
            return;
        }

        pinpoint.setOffsets(FORWARD_POD_SIDEWAYS_OFFSET_INCHES,
                SIDEWAYS_POD_FORWARD_OFFSET_INCHES, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(FORWARD_POD_DIRECTION, SIDEWAYS_POD_DIRECTION);

        // Zero the position and the heading, and recalibrate the gyro.
        pinpoint.resetPosAndIMU();
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the Pinpoint what to do
    // ==================================================================

    /*
     * Call this ONCE per loop(). It reads the Pinpoint and saves the newest answers.
     */
    public void update() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.update();

        xInches = pinpoint.getPosX(DistanceUnit.INCH) * DISTANCE_CALIBRATION_MULTIPLIER;
        yInches = pinpoint.getPosY(DistanceUnit.INCH) * DISTANCE_CALIBRATION_MULTIPLIER;
        headingDegrees = pinpoint.getHeading(AngleUnit.DEGREES);
        xSpeedInchesPerSecond = pinpoint.getVelX(DistanceUnit.INCH) * DISTANCE_CALIBRATION_MULTIPLIER;
        ySpeedInchesPerSecond = pinpoint.getVelY(DistanceUnit.INCH) * DISTANCE_CALIBRATION_MULTIPLIER;
        headingSpeedDegreesPerSecond = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        if (INVERT_HEADING) {
            headingDegrees = -headingDegrees;
            headingSpeedDegreesPerSecond = -headingSpeedDegreesPerSecond;
        }
    }

    /*
     * Makes the robot's CURRENT spot the new start point: X = 0, Y = 0, heading = 0. This is
     * quick and safe any time, even while driving. It does NOT recalibrate the gyro.
     */
    public void resetPose() {
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.DEGREES, 0.0));
        }
        xInches = 0.0;
        yInches = 0.0;
        headingDegrees = 0.0;
        xSpeedInchesPerSecond = 0.0;
        ySpeedInchesPerSecond = 0.0;
        headingSpeedDegreesPerSecond = 0.0;
    }

    /*
     * Zeros the position AND recalibrates the gyro. The robot must be STILL (the gyro needs about
     * a quarter of a second to settle). Use it if the heading has drifted while the robot sat.
     */
    public void recalibrate() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        }
        xInches = 0.0;
        yInches = 0.0;
        headingDegrees = 0.0;
        xSpeedInchesPerSecond = 0.0;
        ySpeedInchesPerSecond = 0.0;
        headingSpeedDegreesPerSecond = 0.0;
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks where the robot is
    // ==================================================================

    /*
     * Inches FORWARD of the start point. Forward is POSITIVE.
     */
    public double getXInches() {
        return xInches;
    }

    /*
     * Inches LEFT of the start point. Left is POSITIVE.
     */
    public double getYInches() {
        return yInches;
    }

    /*
     * Which way the robot faces, in degrees, from -180 to 180. Turning LEFT (counterclockwise) is
     * POSITIVE. Zero is the way the robot faced at the start.
     */
    public double getHeadingDegrees() {
        return headingDegrees;
    }

    /*
     * How fast the robot is going forward (positive) or backward (negative), in inches per second.
     */
    public double getXSpeedInchesPerSecond() {
        return xSpeedInchesPerSecond;
    }

    /*
     * How fast the robot is going left (positive) or right (negative), in inches per second.
     */
    public double getYSpeedInchesPerSecond() {
        return ySpeedInchesPerSecond;
    }

    /*
     * How fast the robot is turning, in degrees per second. Turning LEFT is POSITIVE.
     */
    public double getHeadingSpeedDegreesPerSecond() {
        return headingSpeedDegreesPerSecond;
    }

    /*
     * The raw tick counts of the two pods. They ignore every correction in this class (the
     * offsets, the multiplier, the heading flip) AND the pod direction setting. Tested on the
     * robot: a sideways pod set to REVERSED, with Y correct (+ going left), shows its ticks going
     * DOWN. So use them only to see that each pod is counting at all. Judge the DIRECTIONS by
     * getXInches() and getYInches(). Zero when there is no Pinpoint.
     */
    public int getForwardPodTicks() {
        if (pinpoint == null) {
            return 0;
        }
        return pinpoint.getEncoderX();
    }

    public int getSidewaysPodTicks() {
        if (pinpoint == null) {
            return 0;
        }
        return pinpoint.getEncoderY();
    }

    /*
     * True when the robot's configuration has a device with PINPOINT_NAME.
     */
    public boolean isAvailable() {
        return pinpoint != null;
    }

    /*
     * The Pinpoint's own report on itself: READY, CALIBRATING, NOT_READY, or one of the FAULT_
     * values. When the device is missing we say NOT_READY.
     */
    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        if (pinpoint == null) {
            return GoBildaPinpointDriver.DeviceStatus.NOT_READY;
        }
        return pinpoint.getDeviceStatus();
    }

    /*
     * True only when the Pinpoint is there AND says READY. Check this before trusting the pose.
     */
    public boolean isReady() {
        return pinpoint != null && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /*
     * How many times per second the Pinpoint itself updates. Zero when there is no Pinpoint.
     */
    public double getPinpointFrequencyHz() {
        if (pinpoint == null) {
            return 0.0;
        }
        return pinpoint.getFrequency();
    }

    /*
     * Shows the Pinpoint's health and where the robot is. Call it after update(). It prints its
     * own heading, so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- ODOMETRY (Pinpoint) ---");

        if (pinpoint == null) {
            telemetry.addData("Pinpoint", "NOT FOUND. Add an I2C device named \"%s\" to the robot config", PINPOINT_NAME);
            return;
        }

        telemetry.addData("Pinpoint status", getStatusText());
        telemetry.addData("Pose", "X %.1f in (+ forward)   Y %.1f in (+ left)   Heading %.1f deg (+ left)",
                xInches, yInches, headingDegrees);
        telemetry.addData("Speed", "X %.1f in/s   Y %.1f in/s   Turn %.1f deg/s",
                xSpeedInchesPerSecond, ySpeedInchesPerSecond, headingSpeedDegreesPerSecond);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    /*
     * The status as a sentence a driver can act on.
     */
    private String getStatusText() {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
            return "READY";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.CALIBRATING) {
            return "CALIBRATING (hold the robot still)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
            return "NOT READY (it may still be starting up)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED) {
            return "FAULT: no pods detected (check both pod cables)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED) {
            return "FAULT: forward (X) pod not detected (check its cable)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED) {
            return "FAULT: sideways (Y) pod not detected (check its cable)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
            return "FAULT: bad read (check the I2C cable)";
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY) {
            return "FAULT: gyro runaway (recalibrate with the robot still)";
        }
        return status.toString();
    }
}
