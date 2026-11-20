package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Blue E6 - 3 Volley
 * BLUE alliance. Starts on tile E6 (one tile to the robot's left of D6), touching the REAR wall, facing down at the hive.
 * Shoots, picks 2 flowers, shoots after each, then parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Blue E6 - 3 Volley", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoBlueE6ThreeVolley extends BiobuzzAuto {
    public AutoBlueE6ThreeVolley() {
        super(VisionSubsystem.Alliance.BLUE, StartSpot.LEFT, Routine.THREE_VOLLEY);
    }
}
