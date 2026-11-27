package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BiobuzzTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D6 - Garden D1  (issue #156)
 * BLUE alliance. Starts on tile D6, touching the REAR wall, facing down at the hive.
 *   1. Shoots the 4 preloaded POLLEN, faster than "Blue D6 - Shoot Park" does, then the flywheel stops.
 *   2. Rotates left and drives along the rear wall, facing right with the left bumper along the
 *      wall, into our GARDEN in the top-right corner. It sweeps down the line of POLLEN with the
 *      intake on.
 *   3. As it leaves the garden the flywheel revs up to 1200 RPM. The robot goes down the right side
 *      of the hive to tile D1 (turning left to face up) and shoots again, facing up at the hive.
 *   4. Parks in our loading zone.
 * If it runs out of time after the garden it skips the D1 shot and parks right away.
 * Steps 1 to 3 follow the coach's comment on issue #156 after the first field test.
 * It has not run on a robot since that change. The garden and D1 numbers are starting guesses (see
 * the TODOs in BiobuzzAuto.java). Run it in STEP mode first (press A during INIT).
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue D6 - Garden D1", group = "Auto - Blue", preselectTeleOp = BiobuzzTeleOp.TELEOP_NAME)
public class AutoBlueD6GardenD1 extends BiobuzzAuto {
    public AutoBlueD6GardenD1() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.MAIN, Routine.GARDEN_D1);
    }
}
