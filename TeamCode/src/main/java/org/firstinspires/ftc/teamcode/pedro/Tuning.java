package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

import java.util.function.Supplier;

/*
 * Pedro AutoTune menu
 * Each method marked @Tuner below becomes one entry on the AutoTune web page. You do NOT run
 * anything from the Driver Station list. The steps are:
 *
 *   1. Build and deploy the code to the Control Hub.
 *   2. Connect the laptop to the robot's wifi (the Control Hub's own network).
 *   3. Open http://192.168.43.1:10158 in a browser and pick a procedure.
 *   4. Follow the page. When it finishes, open its "Java" tab and paste the result into
 *      Constants.java.
 *
 * DO THE PROCEDURES IN THIS ORDER (one at a time, on the floor, with a fresh battery):
 *   1. Mecanum Procedure    -> checks the motor names and directions.
 *   2. Pinpoint Procedure   -> checks the pod type, directions and offsets.
 *   3. Tests > Localization -> push the robot: forward must raise X, left must raise Y.
 *   4. Foresight Procedure  -> the long one. The robot DRIVES ITSELF, so clear 8 feet in front
 *                              and to the left, and keep a hand on the stop button.
 *   5. Paste the Foresight numbers into Constants.foresightConfig and set
 *      Constants.FORESIGHT_TUNED = true.
 *   6. Tests > Line Test    -> checks that the tuned robot drives straight and stops where it should.
 *
 * Full guide: TeamCode/docs/pedro-setup-and-auto-guide.md
 *
 * This file is meant to stay short: the real work is inside the procedures/ folder, which is
 * Pedro's own code copied from the official Quickstart. Do not edit those files.
 */
public class Tuning {

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }

    // The tests can check the drivetrain, the localizer and the Foresight algorithm together.
    // Before Foresight is tuned there is nothing to test it with, so we pass nothing (null) and
    // only the Localization, Pose and Driving tests will work. Constants.FORESIGHT_TUNED switches
    // the Hold, Line and Curve tests on.
    @Tuner
    public static Procedure tests() {
        Supplier<Algorithm> algorithmSupplier = null;
        if (Constants.FORESIGHT_TUNED) {
            algorithmSupplier = () -> new Foresight(Constants.foresightConfig);
        }

        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                algorithmSupplier
        );
    }
}
