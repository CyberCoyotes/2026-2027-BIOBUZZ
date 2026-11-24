package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D1 - Shoot Park
 * BLUE alliance. Starts on tile D1 (straight across from D6), touching the AUDIENCE wall right next to the flower, FACING THE WALL (the front of the robot touches it, so the hive is behind the robot; issue #121).
 * Backs up off the wall, turns left 120 degrees so the turret can see the hive, shoots the 4 preloaded POLLEN, then parks in our loading zone.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue D1 - Shoot Park", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoBlueD1ShootPark extends BiobuzzAuto {
    public AutoBlueD1ShootPark() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.FLOWER, Routine.SHOOT_PARK);
    }
}
