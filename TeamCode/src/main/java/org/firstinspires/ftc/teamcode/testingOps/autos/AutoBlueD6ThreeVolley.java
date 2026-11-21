package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D6 - 3 Volley
 * BLUE alliance. Starts on tile D6, touching the REAR wall, facing down at the hive.
 * Shoots, picks 2 flowers (shooting after each), then parks.
 * It runs past 30 s (see README.md), so it is @Disabled to keep the Driver Station list short.
 * To bring it back, put // in front of the @Disabled line below and redeploy.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Disabled
@Autonomous(name = "Blue D6 - 3 Volley", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoBlueD6ThreeVolley extends BiobuzzAuto {
    public AutoBlueD6ThreeVolley() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.MAIN, Routine.THREE_VOLLEY);
    }
}
