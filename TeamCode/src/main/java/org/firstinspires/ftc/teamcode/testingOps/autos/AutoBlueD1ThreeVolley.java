package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D1 - 3 Volley
 * BLUE alliance. Starts on tile D1 (straight across from D6), touching the AUDIENCE wall right next to the flower, facing up at the hive.
 * Shoots, picks 2 flowers (shooting after each), then parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue D1 - 3 Volley", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoBlueD1ThreeVolley extends BiobuzzAuto {
    public AutoBlueD1ThreeVolley() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.FLOWER, Routine.THREE_VOLLEY);
    }
}
