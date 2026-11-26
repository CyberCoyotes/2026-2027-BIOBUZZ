package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BiobuzzTeleOp;
import org.firstinspires.ftc.teamcode.auto.testing.BiobuzzAuto;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D6 - Move Park
 * BLUE alliance. Starts on tile D6, touching the REAR wall, facing down at the hive.
 * Drives to our loading zone and parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue D6 - Move Park", group = "Auto - Blue", preselectTeleOp = BiobuzzTeleOp.TELEOP_NAME)
public class AutoBlueD6MovePark extends BiobuzzAuto {
    public AutoBlueD6MovePark() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.MAIN, Routine.MOVE_PARK);
    }
}
