package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BiobuzzTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D6 - Garden D1  (issue #156)
 * BLUE alliance. Starts on tile D6, touching the REAR wall, facing down at the hive.
 *   1. Shoots the 4 preloaded POLLEN (the same shot as "Blue D6 - Shoot Park").
 *   2. Drives along the rear wall to our GARDEN in the top-right corner, turning around on the way
 *      so the intake faces the wall, and takes in the POLLEN that sit there.
 *   3. Backs down the right side of the hive to tile D1 and shoots again, facing up at the hive.
 *   4. Parks in our loading zone.
 * If it runs out of time after the garden it skips the D1 shot and parks right away.
 * NEW: it has not run on a robot yet. The garden and D1 numbers are starting guesses (see the
 * TODOs in BiobuzzAuto.java). Run it in STEP mode first (press A during INIT).
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue D6 - Garden D1", group = "Auto - Blue", preselectTeleOp = BiobuzzTeleOp.TELEOP_NAME)
public class AutoBlueD6GardenD1 extends BiobuzzAuto {
    public AutoBlueD6GardenD1() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.MAIN, Routine.GARDEN_D1);
    }
}
