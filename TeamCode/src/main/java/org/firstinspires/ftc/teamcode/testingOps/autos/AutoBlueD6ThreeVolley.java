package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue D6 - 3 Volley
 * BLUE alliance. Starts on tile D6, touching the REAR wall, facing down at the hive.
 * Shoots, picks 2 flowers (shooting after each), then parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue D6 - 3 Volley", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoBlueD6ThreeVolley extends BiobuzzAuto {
    public AutoBlueD6ThreeVolley() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.MAIN, Routine.THREE_VOLLEY);
    }
}
