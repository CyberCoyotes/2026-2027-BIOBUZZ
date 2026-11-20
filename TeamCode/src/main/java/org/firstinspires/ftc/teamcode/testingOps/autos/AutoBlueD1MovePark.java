package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D1 - Move Park
 * BLUE alliance. Starts on tile D1 (straight across from D6), touching the AUDIENCE wall right next to the flower, facing up at the hive.
 * Drives to our loading zone and parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue D1 - Move Park", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoBlueD1MovePark extends BiobuzzAuto {
    public AutoBlueD1MovePark() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.FLOWER, Routine.MOVE_PARK);
    }
}
