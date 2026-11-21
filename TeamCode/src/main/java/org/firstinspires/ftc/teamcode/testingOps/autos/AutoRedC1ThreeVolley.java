package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Red C1 - 3 Volley
 * RED alliance. Starts on tile C1, touching the AUDIENCE wall, facing up at the hive.
 * Shoots, picks 2 flowers (shooting after each), then parks.
 * It runs past 30 s (see README.md), so it is @Disabled to keep the Driver Station list short.
 * To bring it back, put // in front of the @Disabled line below and redeploy.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Disabled
@Autonomous(name = "Red C1 - 3 Volley", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoRedC1ThreeVolley extends BiobuzzAuto {
    public AutoRedC1ThreeVolley() {
        super(VisionSubsystem.Alliance.RED, StartSpot.MAIN, Routine.THREE_VOLLEY);
    }
}
