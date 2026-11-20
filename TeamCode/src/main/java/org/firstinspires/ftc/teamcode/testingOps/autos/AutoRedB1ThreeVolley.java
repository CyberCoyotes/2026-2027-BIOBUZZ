package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Red B1 - 3 Volley
 * RED alliance. Starts on tile B1 (one tile to the robot's left of C1), touching the AUDIENCE wall, facing up at the hive.
 * Shoots, picks 2 flowers, shoots after each, then parks.
 * All the code is in BiobuzzAuto. This file only picks the alliance, the start spot and the routine.
 */
@Autonomous(name = "Red B1 - 3 Volley", group = "Auto", preselectTeleOp = "TeleOp - Alpha 15")
public class AutoRedB1ThreeVolley extends BiobuzzAuto {
    public AutoRedB1ThreeVolley() {
        super(VisionSubsystem.Alliance.RED, StartSpot.LEFT, Routine.THREE_VOLLEY);
    }
}
