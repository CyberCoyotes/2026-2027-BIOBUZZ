package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D6 - Shoot Park
 * BLUE alliance. Starts on tile D6, touching the REAR wall, facing down at the hive.
 * Shoots the 4 preloaded POLLEN, then parks in our loading zone.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue D6 - Shoot Park", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoBlueD6ShootPark extends BiobuzzAuto {
    public AutoBlueD6ShootPark() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.CENTER, Routine.SHOOT_PARK);
    }
}
