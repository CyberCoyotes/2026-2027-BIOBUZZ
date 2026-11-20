package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Red C6 - 3 Volley
 * RED alliance. Starts on tile C6 (straight across from C1), touching the REAR wall right next to the flower, facing down at the hive.
 * Shoots, picks 2 flowers (shooting after each), then parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Red C6 - 3 Volley", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoRedC6ThreeVolley extends BiobuzzAuto {
    public AutoRedC6ThreeVolley() {
        super(VisionSubsystem.Alliance.RED, StartSpot.FLOWER, Routine.THREE_VOLLEY);
    }
}
