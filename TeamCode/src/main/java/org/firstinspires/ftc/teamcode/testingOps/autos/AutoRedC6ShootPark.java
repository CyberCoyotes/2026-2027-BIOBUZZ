package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Red C6 - Shoot Park
 * RED alliance. Starts on tile C6 (straight across from C1), touching the REAR wall right next to the flower, facing down at the hive.
 * Shoots the 4 preloaded POLLEN, then parks in our loading zone.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Red C6 - Shoot Park", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoRedC6ShootPark extends BiobuzzAuto {
    public AutoRedC6ShootPark() {
        super(VisionSubsystem.Alliance.RED, StartSpot.FLOWER, Routine.SHOOT_PARK);
    }
}
