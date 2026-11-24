package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Red C6 - Move Park
 * RED alliance. Starts on tile C6 (straight across from C1), touching the REAR wall right next to the flower, FACING THE WALL (the front of the robot touches it, so the hive is behind the robot; issue #121).
 * Drives to our loading zone and parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Red C6 - Move Park", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoRedC6MovePark extends BiobuzzAuto {
    public AutoRedC6MovePark() {
        super(VisionSubsystem.Alliance.RED, StartSpot.FLOWER, Routine.MOVE_PARK);
    }
}
