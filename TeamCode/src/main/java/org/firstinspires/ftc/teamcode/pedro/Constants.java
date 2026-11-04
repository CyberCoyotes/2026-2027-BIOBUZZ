package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Pedro Pathing constants (Pedro 3)
 * This file tells Pedro about OUR robot. It has three groups of numbers, and one method that
 * builds the Follower (the object that drives the robot along a path):
 *
 *   1. drivetrainConfig : the four drive motors (names and directions).
 *   2. localizerConfig  : the goBILDA Pinpoint (name, where the pods sit, which way they count).
 *   3. foresightConfig  : how Pedro brakes and corrects. THESE MUST COME FROM AUTOTUNE.
 *
 * Auto OpModes call Constants.create(hardwareMap) in init() and get a ready Follower back.
 *
 * ONE RULE: in an Auto OpMode, Pedro owns the drive motors and the Pinpoint. Never use
 * DriveSubsystem or PinpointSubsystem in the same OpMode as a Pedro Follower.
 *
 * NOTE ON LAMBDAS: the "c -> { ... }" blocks below are how Pedro wants its settings written, so
 * we use them here. They are the ONLY place our code uses lambdas. Fill in the blanks the same
 * way and you will not need to understand them yet.
 *
 * HOW THIS FILE GETS FINISHED (full steps: TeamCode/docs/pedro-setup-and-auto-guide.md)
 *   - The drivetrain and Pinpoint numbers below are already filled in from DriveSubsystem and
 *     PinpointSubsystem. The Pinpoint AutoTune step double-checks them.
 *   - The foresightConfig numbers are NOT ours yet. They are an example from another team's
 *     robot. Run the Foresight AutoTune on each robot, paste its numbers over them, and only
 *     then change FORESIGHT_TUNED to true.
 */
public class Constants {

    // ------------------------------------------------------------------
    // The safety switch
    // ------------------------------------------------------------------
    // false = the foresightConfig numbers below are still the EXAMPLE numbers from another robot.
    // Auto OpModes check this and refuse to drive while it is false, because wrong braking
    // numbers can make a tall, fast robot overshoot or slam into a wall.
    // Change it to true ONLY AFTER you pasted this robot's AutoTune numbers into foresightConfig.
    // Each robot (practice bot, 11940, 22091) needs its own tune.
    public static final boolean FORESIGHT_TUNED = false;

    // ------------------------------------------------------------------
    // 1. Drive motors
    // ------------------------------------------------------------------
    // The names MUST match the robot configuration, and they are the same names DriveSubsystem uses.
    // The directions are also the same as DriveSubsystem: left side REVERSE, right side FORWARD,
    // so a positive power drives the robot forward. The Mecanum AutoTune can check them.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("left_front");
                c.backLeftName.set("left_rear");
                c.frontRightName.set("right_front");
                c.backRightName.set("right_rear");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                // The wheels brake when the sticks are released in TeleOp. Pedro's own Follower
                // does its braking itself, so this only matters if we drive by hand through Pedro.
                c.manualBrakeMode.set(true);
            }
    );

    // ------------------------------------------------------------------
    // 2. Pinpoint odometry
    // ------------------------------------------------------------------
    // These come from PinpointSubsystem, which we already tested on the robot. If the Pinpoint
    // AutoTune ever finds different numbers, change them in BOTH places so TeleOp and Auto agree.
    //
    // Pedro calls the FORWARD pod the "x pod" and the SIDEWAYS pod the "y pod".
    //   xPodOffset : how far the forward pod sits to the side of the robot's turning center.
    //                LEFT is positive. Ours is 3.75 inches to the RIGHT, so -3.75.
    //   yPodOffset : how far the sideways pod sits ahead of (+) or behind (-) the turning center.
    //                Ours is centered front to back, so 0.0.
    // Offsets are in inches.
    //
    // podType: the DEFAULT in Pedro is the 4-bar pod, but ours are the spring-loaded SWINGARM pods,
    // so we MUST set it here. A wrong pod type makes every distance read wrong by a steady amount.
    //
    // Directions: the rule is "X goes UP when pushed forward, Y goes UP when pushed left".
    // Tested on the robot with PinpointAlpha: forward pod FORWARD, sideways pod REVERSED.
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
                c.xPodOffset.set(-3.75);
                c.yPodOffset.set(0.0);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            }
    );

    // ------------------------------------------------------------------
    // 3. Foresight (the path-following algorithm)
    // ------------------------------------------------------------------
    // !!! EXAMPLE NUMBERS FROM ANOTHER TEAM'S ROBOT (copied from Pedro's documentation). !!!
    // They are only here so the file compiles. They are NOT tuned for our robots.
    // Replace this whole block with the "Java" tab output of the Foresight AutoTune, then set
    // FORESIGHT_TUNED to true at the top of this file.
    //
    // The end-of-path settings (how close counts as "arrived") are left at Pedro's defaults:
    //   translationalConstraint 0.1 in, headingConstraint 0.007 rad, velocityConstraint 0.1 in/s,
    //   timeoutConstraint 100 ms. If a path takes too long to finish, those are the numbers to look at.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3);
                Controller secondaryTranslationalForward = Controller.proportional(0.1);
                Controller primaryTranslationalLateral = Controller.proportional(0.3);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
                c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));

                c.headingFeedback.set(Controller.proportional(5.258721785960744));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));

                c.maxAchievableForwardVelocity.set(72.72923108818539);
                c.maxAchievableStrafeVelocity.set(52.34323936525474);
                c.naturalForwardDeceleration.set(85.01144677379789);
                c.naturalStrafeDeceleration.set(104.49787535782846);
            }
    );

    // ------------------------------------------------------------------
    // Building the Follower
    // ------------------------------------------------------------------
    // A Follower needs three parts: where the robot is (localizer), how to move the wheels
    // (drivetrain), and how to follow a path (Foresight). Make sure the localizer and drivetrain
    // here match the configs above.
    //
    // Creating the Pinpoint localizer recalibrates the Pinpoint's gyro and waits half a second,
    // so the robot must sit STILL while an Auto OpMode is in INIT.
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
