package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BiobuzzTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D1 - 3 Volley
 * BLUE alliance. Starts on tile D1 (straight across from D6), touching the AUDIENCE wall right next to the flower, facing up at the hive.
 * Shoots, picks 2 flowers (shooting after each), then parks.
 * It runs past 30 s (see README.md), so it is @Disabled to keep the Driver Station list short.
 * To bring it back, put // in front of the @Disabled line below and redeploy.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Disabled
@Autonomous(name = "Blue D1 - 3 Volley", group = "Auto - Blue", preselectTeleOp = BiobuzzTeleOp.TELEOP_NAME)
public class AutoBlueD1ThreeVolley extends BiobuzzAuto {
    public AutoBlueD1ThreeVolley() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.FLOWER, Routine.THREE_VOLLEY);
    }
}
