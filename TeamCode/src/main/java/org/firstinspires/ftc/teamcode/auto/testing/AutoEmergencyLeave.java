package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.AutoHandoff;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

/*
 * Drive Forward (the A-0 Emergency Leave Auto)
 * The simplest Auto there is (rung A-0 in TeamCode/docs/biobuzz-auto-strategy.md). The robot drives
 * straight FORWARD 24 inches and stops. That is LEAVE = 3 points, as long as the robot is no
 * longer touching the perimeter wall when AUTO ends.
 *
 * WHEN TO USE IT
 *   - Pedro Pathing is not tuned yet (FORESIGHT_TUNED is false), so AutoLeaveAndPark will not
 *     drive. This is the stopgap until it is tuned.
 *   - Or something else is broken at an event (the camera is dead, Pedro is not working) and we
 *     want to be sure of the 3 LEAVE points.
 *
 * HOW IT WORKS
 *   It uses DriveSubsystem (the same class as the TeleOp) to drive, and PinpointSubsystem to
 *   MEASURE how far the robot has gone. No Pedro, no camera. It drives until the Pinpoint says
 *   the robot is 24 inches from where it started, then stops. Because it measures the distance,
 *   a weak battery or a slippery floor does not change how far it goes.
 *
 *   TWO SAFETY NETS (so a broken Pinpoint can never leave us driving forever):
 *     1. SAFETY_TIMEOUT_SECONDS: if the robot has not reached 24 inches by then, it stops anyway.
 *     2. If the Pinpoint is NOT READY when you press START, there is no distance to measure, so
 *        the robot drives for BACKUP_DRIVE_SECONDS instead (the old timed drive). The screen says
 *        "BACKUP: timed drive" when this happens. Fix the Pinpoint if you can, but the 3 LEAVE
 *        points are still on the table.
 *   It does NOT park in the LOADING ZONE. The zone is too far away for a straight drive.
 *   Parking needs Pedro.
 *
 * WHERE THE ROBOT STARTS
 *   The same start spot as the other Autos: back against the wall, facing the hive. Blue is tile
 *   D6 (top wall, facing down), red is tile C1 (bottom wall, facing up). "Forward" is always
 *   toward the hive, so the same OpMode works for both alliances. Nothing needs to be chosen.
 *   The robot must be inside the 18 inch cube, touching the wall, not in the LOADING ZONE and not
 *   touching a flower. (All of that is in pedro/FieldConstants.java.)
 *   The hive is about 40 inches from the wall on the real field, so 24 inches leaves a safe gap.
 *
 * BEFORE YOU PRESS INIT
 *   The Pinpoint's gyro calibrates at INIT, so the robot must be STILL (same as every Pinpoint
 *   OpMode). Wait for "Pinpoint: READY" on the screen before you press START.
 *
 * HOW TO TUNE IT (all at the top of this file)
 *   1. Put the robot against the wall, with a clear floor in front of it. Keep DRIVE_POWER low.
 *   2. Run it and measure how far the robot really moved from its start. It should be 24 inches.
 *      The robot keeps rolling a little after the motors stop, so it may go a bit over. If it
 *      does, make DRIVE_DISTANCE_INCHES a little smaller (or DRIVE_POWER lower) and run again.
 *   3. Run it 5 times in a row and write the distances in
 *      TeamCode/docs/biobuzz-test-and-match-log.md.
 *   4. To check the backup, unplug the Pinpoint (or pick a robot config without it), run it, and
 *      measure. Change BACKUP_DRIVE_SECONDS until it also goes about 24 inches.
 *
 * Nothing moves during INIT (rule G403). The robot only moves after START. Stop it any time with
 * the Driver Station STOP button.
 */
@Autonomous(name = "Drive Forward", group = "Auto",
        preselectTeleOp = "TeleOp - Alpha 15")
public class AutoEmergencyLeave extends OpMode {

    // ------------------------------------------------------------------
    // Settings (change these to change the Auto)
    // ------------------------------------------------------------------
    // How hard to drive, from 0.0 to 1.0. Keep it low: a tall robot that stops from full speed can
    // tip, and a low power is easier to repeat. 0.30 is a starting guess.
    private static final double DRIVE_POWER = 0.30;

    // How far to drive, in inches, measured by the Pinpoint. 24 inches is clearly off the wall and
    // still well short of the hive (about 40 inches away).
    // TODO: measure on the robot how far it really goes (it coasts a little after the stop).
    private static final double DRIVE_DISTANCE_INCHES = 24.0;

    // If the robot has not gone DRIVE_DISTANCE_INCHES after this many seconds, stop anyway. This is
    // only a safety net for a Pinpoint that stops counting. 2.0 is a starting guess: at 0.30 power
    // the robot should need about 1.6 seconds for 24 inches.
    // TODO: time a normal run on the robot and set this a bit longer than that.
    private static final double SAFETY_TIMEOUT_SECONDS = 2.0;

    // Used ONLY when the Pinpoint is not READY at START: drive this many seconds instead of a
    // distance. 1.6 is a starting guess (the old 0.80 seconds went about half as far as we want).
    // TODO: measure on the robot with the Pinpoint unplugged, then tune it to about 24 inches.
    private static final double BACKUP_DRIVE_SECONDS = 1.6;

    // ------------------------------------------------------------------
    // The steps of this Auto. The robot is always in exactly one of these.
    // ------------------------------------------------------------------
    private enum AutoState {
        DRIVING,
        DONE
    }

    // ------------------------------------------------------------------
    // Things this OpMode uses
    // ------------------------------------------------------------------
    private DriveSubsystem driveSubsystem;
    private PinpointSubsystem pinpointSubsystem;

    private AutoState autoState = AutoState.DONE;

    // True when the Pinpoint was READY at START, so we can drive by distance.
    // False means we are in the BACKUP timed drive.
    private boolean drivingByDistance = false;

    // How far the robot is from its start spot, in inches (straight-line distance).
    private double distanceDrivenInches = 0.0;

    // Times the drive. (A timer is used instead of sleep() so loop() keeps running.)
    private final ElapsedTime driveTimer = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The wheels start stopped. Nothing moves in INIT.
        driveSubsystem = new DriveSubsystem(hardwareMap);

        // Creating the Pinpoint zeros its position and calibrates its gyro, so the robot must be
        // STILL now.
        pinpointSubsystem = new PinpointSubsystem(hardwareMap);

        telemetry.addLine("Ready. Put the robot on its start spot against the wall, then press START.");
        addPinpointStatusTelemetry();
        addSettingsTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Keep reading the Pinpoint so the screen shows whether it is READY yet.
        pinpointSubsystem.update();
        addPinpointStatusTelemetry();
        addSettingsTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        // This Auto does not know which alliance we are, so forget any alliance an earlier Auto
        // saved. The TeleOp then starts on BLUE, like before, and the operator presses START to
        // switch it. (Better than starting on the WRONG alliance from an old Auto.)
        AutoHandoff.clear();

        // Decide how we will measure this drive. Check READY here, once, so the answer cannot
        // change halfway through the drive.
        drivingByDistance = pinpointSubsystem.isReady();

        // Make the robot's current spot the zero point, so "distance" means "from the start".
        pinpointSubsystem.resetPose();

        driveTimer.reset();
        autoState = AutoState.DRIVING;
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // 1. Read sensors. getXInches() is forward and getYInches() is left, so the straight-line
        //    distance from the start is the hypotenuse. (This way it still works if the robot
        //    drifts a little sideways.)
        pinpointSubsystem.update();
        distanceDrivenInches = Math.hypot(pinpointSubsystem.getXInches(),
                pinpointSubsystem.getYInches());

        // 2. Decide what to do next, and tell the drivetrain.
        switch (autoState) {
            case DRIVING:
                // forward = DRIVE_POWER, strafe = 0, turn = 0: straight ahead.
                driveSubsystem.drive(DRIVE_POWER, 0.0, 0.0);
                if (isDriveFinished()) {
                    driveSubsystem.stop();
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                // Keep telling the wheels to stop, so nothing can drift.
                driveSubsystem.stop();
                break;
        }

        // 3. Telemetry
        telemetry.addData("State", autoState);
        if (drivingByDistance) {
            telemetry.addData("Mode", "Pinpoint distance");
            telemetry.addData("Distance driven (in)", "%.1f of %.1f",
                    distanceDrivenInches, DRIVE_DISTANCE_INCHES);
        } else {
            telemetry.addData("Mode", "BACKUP: timed drive (Pinpoint was not READY)");
        }
        telemetry.addData("Time driving (s)", "%.2f", driveTimer.seconds());
        addSettingsTelemetry();
        driveSubsystem.addTelemetry(telemetry);
        pinpointSubsystem.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        driveSubsystem.stop();
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    /*
     * True when it is time to stop driving.
     */
    private boolean isDriveFinished() {
        if (drivingByDistance) {
            // Normal case: stop at the distance, or at the safety timeout, whichever comes first.
            boolean reachedDistance = distanceDrivenInches >= DRIVE_DISTANCE_INCHES;
            boolean tookTooLong = driveTimer.seconds() >= SAFETY_TIMEOUT_SECONDS;
            return reachedDistance || tookTooLong;
        }

        // Backup case: no distance to measure, so stop on the timer alone.
        return driveTimer.seconds() >= BACKUP_DRIVE_SECONDS;
    }

    private void addPinpointStatusTelemetry() {
        if (pinpointSubsystem.isReady()) {
            telemetry.addData("Pinpoint", "READY: will drive %.0f inches", DRIVE_DISTANCE_INCHES);
        } else {
            telemetry.addData("Pinpoint", "NOT READY: would use BACKUP timed drive (%.1f s)",
                    BACKUP_DRIVE_SECONDS);
        }
    }

    private void addSettingsTelemetry() {
        telemetry.addData("Drive power", "%.2f", DRIVE_POWER);
        telemetry.addData("Drive distance (in)", "%.1f", DRIVE_DISTANCE_INCHES);
    }
}
