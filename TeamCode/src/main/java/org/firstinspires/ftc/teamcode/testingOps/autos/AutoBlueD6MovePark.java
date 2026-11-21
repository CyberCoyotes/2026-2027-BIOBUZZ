package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D6 - Move Park
 * BLUE alliance. Starts on tile D6, touching the REAR wall, facing down at the hive.
 * Drives to our loading zone and parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue D6 - Move Park", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoBlueD6MovePark extends BiobuzzAuto {
    public AutoBlueD6MovePark() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.MAIN, Routine.MOVE_PARK);
    }
}
