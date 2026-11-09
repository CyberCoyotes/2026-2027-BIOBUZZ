package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Pedro Forward and Back, Alpha
 * Our first Pedro Pathing autonomous. The robot drives straight FORWARD, pauses, then drives
 * straight BACK to the exact spot it started from. It is a TEST of the Pedro setup (the tuning
 * and the Pinpoint), not a scoring Auto, so it is in the "Testing" group.
 *
 *   1. DRIVE_FORWARD : follow a straight line 24 inches forward.
 *   2. PAUSE         : hold still for 1 second.
 *   3. DRIVE_BACK    : follow a straight line back to the start.
 *   4. DONE          : the robot holds its spot.
 *
 * This OpMode owns NO motors itself. Pedro's Follower drives the four drive motors and reads the
 * Pinpoint. So do NOT use DriveSubsystem or PinpointSubsystem in this OpMode.
 *
 * BEFORE YOU RUN IT (once per robot)
 *   - Do the Pedro tuning in TeamCode/docs/pedro-setup-and-auto-guide.md, paste the numbers into
 *     pedro/Constants.java, and set FORESIGHT_TUNED to true. Until then this OpMode REFUSES to
 *     drive and says so on the screen.
 *   - Clear at least 3 feet of floor in FRONT of the robot.
 *
 * HOW TO RUN IT
 *   1. Set the robot on the floor, facing the direction it should drive, and leave it STILL.
 *      (INIT recalibrates the Pinpoint's gyro, so a robot that moves during INIT will be off.)
 *   2. Select "Pedro Forward Back - Alpha" and press INIT. Wait for "Pinpoint ready" below.
 *   3. Press START.
 *
 * WHAT TO WATCH FOR
 *   - The robot should come back to within about an inch of where it started and face the same
 *     way. Put a piece of tape at each wheel before START and check afterward.
 *   - "Pose" on the screen is where Pedro THINKS the robot is. If the robot is not really where
 *     the screen says, the Pinpoint settings are wrong (see Constants.java), not this OpMode.
 *   - Run it 5 times in a row. Write each run in TeamCode/docs/biobuzz-test-and-match-log.md.
 *
 * COORDINATES (Pedro's, not the FTC field's): X is straight ahead when the heading is 0 degrees,
 * Y is to the LEFT, and turning counterclockwise (left) is a positive heading. Because we start
 * at (0, 0) facing 0 degrees, "forward" is simply +X.
 */
@Autonomous(name = "Pedro Forward Back - Alpha", group = "Testing")
public class PedroForwardBackAlpha extends OpMode {

    // ------------------------------------------------------------------
    // Settings (change these to change the test)
    // ------------------------------------------------------------------
    // How far to drive forward, in inches. The robot comes back the same distance.
    // Start small for the first run on a new tune, then try larger numbers.
    private static final double DRIVE_DISTANCE_INCHES = 24.0;

    // How long to hold still between the two drives, in seconds.
    private static final double PAUSE_SECONDS = 1.0;

    // Where Pedro should think the robot starts. (0, 0) facing 0 degrees keeps the numbers simple:
    // forward is +X. This is a test, so it does not need to match a real spot on the field.
    private static final double START_X_INCHES = 0.0;
    private static final double START_Y_INCHES = 0.0;
    private static final double START_HEADING_DEGREES = 0.0;

    // ------------------------------------------------------------------
    // The steps of this Auto. The robot is always in exactly one of these.
    // ------------------------------------------------------------------
    private enum AutoState {
        DRIVE_FORWARD,
        PAUSE,
        DRIVE_BACK,
        DONE
    }

    // ------------------------------------------------------------------
    // Things this OpMode uses
    // ------------------------------------------------------------------
    private Follower follower;

    // A PoseFactory makes poses (an X, a Y and a heading) from numbers in DEGREES.
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Pose startPose;
    private Pose forwardPose;

    // The two paths are built once in init(), never inside loop().
    private Path forwardPath;
    private Path backPath;

    private AutoState autoState = AutoState.DONE;

    // Times the pause between the drives. A timer is used instead of sleep() so loop() keeps
    // running (and the Follower keeps holding the robot's position) during the pause.
    private final ElapsedTime pauseTimer = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Build the two poses: where we start, and where we turn around.
        startPose = poseFactory.of(START_X_INCHES, START_Y_INCHES, START_HEADING_DEGREES);
        forwardPose = poseFactory.of(START_X_INCHES + DRIVE_DISTANCE_INCHES, START_Y_INCHES, START_HEADING_DEGREES);

        // Create the Follower. This recalibrates the Pinpoint, so keep the robot still.
        follower = Constants.create(hardwareMap);

        // Tell Pedro where the robot is standing right now, then read the Pinpoint once.
        follower.setPose(startPose);
        follower.update();

        forwardPath = buildForwardPath();
        backPath = buildBackPath();

        telemetry.addLine("Pinpoint ready. Keep the robot still, then press START.");
        addSafetyTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // The Follower is idle here, so this only reads the Pinpoint and keeps the wheels stopped.
        // It lets you push the robot by hand and watch the pose change.
        follower.update();

        addSafetyTelemetry();
        addPoseTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        if (!Constants.FORESIGHT_TUNED) {
            // Do not drive on another robot's braking numbers. Stay in DONE and show why.
            autoState = AutoState.DONE;
            return;
        }

        autoState = AutoState.DRIVE_FORWARD;
        follower.follow(forwardPath);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Update Pedro. This reads the Pinpoint and sets the wheel powers. It must run
        //    every loop, or the robot does not move (or stops steering).
        follower.update();

        // 2. Decide what to do next.
        switch (autoState) {
            case DRIVE_FORWARD:
                // isBusy() stays true until the robot has arrived and settled.
                if (!follower.isBusy()) {
                    pauseTimer.reset();
                    autoState = AutoState.PAUSE;
                }
                break;

            case PAUSE:
                // The Follower is holding the robot's spot while we wait.
                if (pauseTimer.seconds() >= PAUSE_SECONDS) {
                    follower.follow(backPath);
                    autoState = AutoState.DRIVE_BACK;
                }
                break;

            case DRIVE_BACK:
                if (!follower.isBusy()) {
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                // Nothing left to do. The Follower keeps holding the last pose.
                break;
        }

        // 3. Telemetry
        addSafetyTelemetry();
        telemetry.addData("State", autoState);
        addPoseTelemetry();
        telemetry.addData("Follower mode", follower.mode());
        telemetry.addData("Distance to end of path (in)", "%.2f", follower.distanceToEndpoint());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Put the Follower in IDLE and tell the drivetrain to stop right now. (follower.stop()
        // only changes the mode. The wheels are zeroed by the next update(), and there is none.)
        follower.stop();
        follower.drivetrain.stop();
    }

    // ------------------------------------------------------------------
    // Paths
    // ------------------------------------------------------------------
    // A path is a line (or curve) between poses. ".constant(startPose)" means: keep facing the
    // heading of startPose the whole way, so the robot drives straight and never turns.

    // From the start straight to the turnaround spot.
    private Path buildForwardPath() {
        return Paths.line(startPose, forwardPose).constant(startPose);
    }

    // From the turnaround spot straight back to the start.
    private Path buildBackPath() {
        return Paths.line(forwardPose, startPose).constant(startPose);
    }

    // ------------------------------------------------------------------
    // Telemetry helpers
    // ------------------------------------------------------------------
    private void addSafetyTelemetry() {
        if (Constants.FORESIGHT_TUNED) {
            telemetry.addData("Foresight", "tuned, OK to drive");
        } else {
            telemetry.addLine("!!! NOT TUNED: this OpMode will NOT drive. !!!");
            telemetry.addLine("Tune Foresight (see the Pedro guide), paste the numbers into");
            telemetry.addLine("pedro/Constants.java, then set FORESIGHT_TUNED = true.");
        }
    }

    private void addPoseTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pose X (in)", "%.2f", pose.x());
        telemetry.addData("Pose Y (in)", "%.2f", pose.y());
        telemetry.addData("Pose heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
    }
}
