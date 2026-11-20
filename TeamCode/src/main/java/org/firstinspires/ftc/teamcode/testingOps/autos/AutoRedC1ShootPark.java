package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Red C1 - Shoot Park
 * RED alliance. Starts on tile C1, touching the AUDIENCE wall, facing up at the hive.
 * Shoots the 4 preloaded POLLEN, then parks in our loading zone.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Red C1 - Shoot Park", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoRedC1ShootPark extends BiobuzzAuto {
    public AutoRedC1ShootPark() {
        super(VisionSubsystem.Alliance.RED, StartSpot.MAIN, Routine.SHOOT_PARK);
    }
}
