package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Red B1 - Move Park
 * RED alliance. Starts on tile B1 (one tile to the robot's left of C1), touching the AUDIENCE wall, facing up at the hive.
 * Drives to our loading zone and parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Red B1 - Move Park", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoRedB1MovePark extends BiobuzzAuto {
    public AutoRedB1MovePark() {
        super(VisionSubsystem.Alliance.RED, StartSpot.LEFT, Routine.MOVE_PARK);
    }
}
