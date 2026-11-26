package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BiobuzzTeleOp;
import org.firstinspires.ftc.teamcode.auto.testing.BiobuzzAuto;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Red C1 - Move Park
 * RED alliance. Starts on tile C1, touching the AUDIENCE wall, facing up at the hive.
 * Drives to our loading zone and parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Red C1 - Move Park", group = "Auto - Red", preselectTeleOp = BiobuzzTeleOp.TELEOP_NAME)
public class AutoRedC1MovePark extends BiobuzzAuto {
    public AutoRedC1MovePark() {
        super(VisionSubsystem.Alliance.RED, StartSpot.MAIN, Routine.MOVE_PARK);
    }
}
