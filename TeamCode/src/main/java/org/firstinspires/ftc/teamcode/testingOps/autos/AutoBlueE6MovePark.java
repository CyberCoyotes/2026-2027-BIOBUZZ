package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue E6 - Move Park
 * BLUE alliance. Starts on tile E6 (one tile to the robot's left of D6), touching the REAR wall, facing down at the hive.
 * Drives to our loading zone and parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue E6 - Move Park", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoBlueE6MovePark extends BiobuzzAuto {
    public AutoBlueE6MovePark() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.LEFT, Routine.MOVE_PARK);
    }
}
