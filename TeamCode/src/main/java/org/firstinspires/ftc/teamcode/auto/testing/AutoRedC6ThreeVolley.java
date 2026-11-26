package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BiobuzzTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Red C6 - 3 Volley
 * RED alliance. Starts on tile C6 (straight across from C1), touching the REAR wall right next to the flower, facing down at the hive.
 * Shoots, picks 2 flowers (shooting after each), then parks.
 * It runs past 30 s (see README.md), so it is @Disabled to keep the Driver Station list short.
 * To bring it back, put // in front of the @Disabled line below and redeploy.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Disabled
@Autonomous(name = "Red C6 - 3 Volley", group = "Auto - Red", preselectTeleOp = BiobuzzTeleOp.TELEOP_NAME)
public class AutoRedC6ThreeVolley extends BiobuzzAuto {
    public AutoRedC6ThreeVolley() {
        super(VisionSubsystem.Alliance.RED, StartSpot.FLOWER, Routine.THREE_VOLLEY);
    }
}
