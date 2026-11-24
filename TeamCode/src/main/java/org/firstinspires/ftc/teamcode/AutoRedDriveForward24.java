package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AutoHandoff;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Red - Drive Forward 24in  (a "dumb" Auto: no Pedro, no camera, no field map)
 * RED alliance. The robot drives straight FORWARD 24 inches and stops. That is
 * LEAVE = 3 points, as long as the robot is no longer touching the perimeter wall when AUTO ends.
 *
 * WHY THERE ARE FOUR OF THESE (issue #114)
 *   AutoBlueDriveForward24, AutoRedDriveForward24, AutoBlueDriveReverse24 and
 *   AutoRedDriveReverse24 are plain copies of AutoEmergencyLeave ("Drive Forward"). That is on
 *   purpose: every file stands alone, so each one can be tuned on its own and none can break
 *   another. They differ only in the alliance, the direction, and these comments. If you fix a bug
 *   in one, fix it in the other three and in AutoEmergencyLeave too.
 *
 * WHEN TO USE IT
 *   Use it when the robot starts with its BACK toward a wall, so driving forward
 *   takes it off the wall.
 *   Use it when Pedro Pathing is not tuned yet (FORESIGHT_TUNED is false), or when something
 *   else is broken at an event (the camera is dead, Pedro is not working) and we want to be sure
 *   of the 3 LEAVE points.
 *
 * NO GRID REFERENCES
 *   This Auto does not care which tile or wall the robot starts on. It drives 24 inches in a
 *   straight line from wherever it starts. The drive team must make sure that path is clear: no
 *   hive, no flower, no other robot.
 *
 * HOW IT WORKS
 *   It uses DriveSubsystem (the same class as the TeleOp) to drive, and PinpointSubsystem to
 *   MEASURE how far the robot has gone. No Pedro, no camera. It drives until the Pinpoint says
 *   the robot is close to 24 inches from where it started, then stops. "Close" means 24 minus
 *   COAST_DISTANCE_INCHES: the robot keeps rolling a little after the wheels stop, so the Auto
 *   stops a little EARLY and lets the robot roll the rest of the way. Because it measures the
 *   distance, a weak battery or a slippery floor does not change how far it goes.
 *
 *   TWO SAFETY NETS (so a broken Pinpoint can never leave us driving forever):
 *     1. SAFETY_TIMEOUT_SECONDS: if the robot has not reached 24 inches by then, it stops anyway.
 *     2. If the Pinpoint is NOT READY when you press START, there is no distance to measure, so
 *        the robot drives for BACKUP_DRIVE_SECONDS instead (a timed drive). The screen says
 *        "BACKUP: timed drive" when this happens. Fix the Pinpoint if you can, but the 3 LEAVE
 *        points are still on the table.
 *   It does NOT park in the LOADING ZONE. Parking needs Pedro.
 *
 * WHAT IT TELLS THE TELEOP
 *   At START it saves the alliance (RED) with AutoHandoff, so the TeleOp starts on the
 *   right alliance without anyone pressing a button. (The old "Drive Forward" does not know the
 *   alliance, so after it the TeleOp always started on BLUE.)
 *   It does NOT save a heading, because this Auto does not know which wall you started on. So the
 *   TeleOp uses its own guess for field-centric driving. For RED that guess is that the
 *   robot faces:
 *       UP the field map (toward the rear wall)
 *   A start with the robot's back against the AUDIENCE wall (our usual red start)
 *   matches that guess. If the robot started facing some other way (for example against a side
 *   wall), field-centric driving will be turned until the driver re-aims (D-pad Left, see
 *   BiobuzzTeleOp).
 *
 * BEFORE YOU PRESS INIT
 *   The Pinpoint's gyro calibrates at INIT, so the robot must be STILL (same as every Pinpoint
 *   OpMode). Wait for "Pinpoint: READY" on the screen before you press START.
 *
 * HOW TO TUNE IT (all at the top of this file)
 *   1. Put the robot on a clear floor with 24 inches or more of room. Keep DRIVE_POWER low.
 *   2. Run it and measure with a tape measure how far the robot really moved from its start. It
 *      should be 24 inches. The robot rolls on after the motors stop (the coast), so the Auto
 *      stops early by COAST_DISTANCE_INCHES. If the robot still ends up long, make
 *      COAST_DISTANCE_INCHES bigger by the amount it was over, and run again. If it ends up
 *      short, make it smaller. Leave DRIVE_DISTANCE_INCHES at 24: it is the number in the class
 *      name and the @Autonomous name. If you change DRIVE_POWER, measure again, because a
 *      faster robot rolls farther.
 *   3. Run it 5 times in a row and write the distances in
 *      TeamCode/docs/biobuzz-test-and-match-log.md.
 *   4. To check the backup, unplug the Pinpoint (or pick a robot config without it), run it, and
 *      measure. Change BACKUP_DRIVE_SECONDS until it also goes about 24 inches.
 *
 * Nothing moves during INIT (rule G403). The robot only moves after START. Stop it any time with
 * the Driver Station STOP button.
 */
@Autonomous(name = "Red - Drive Forward 24in", group = "Auto",
        preselectTeleOp = "TeleOp - Alpha 15")
public class AutoRedDriveForward24 extends OpMode {

    // ------------------------------------------------------------------
    // Settings (change these to change the Auto)
    // ------------------------------------------------------------------
    // The alliance this Auto plays. It is saved for the TeleOp at START (see "WHAT IT TELLS THE
    // TELEOP" above). It does not change how the robot drives.
    private static final VisionSubsystem.Alliance ALLIANCE = VisionSubsystem.Alliance.RED;

    // How hard to drive, from 0.0 to 1.0. Keep it low: a tall robot that stops from full speed can
    // tip, and a low power is easier to repeat. 0.30 is a starting guess.
    // TODO: Forward may not go as far as the other direction at the same power, so
    // tune each of the four files on its own.
    private static final double DRIVE_POWER = 0.30;

    // How far to drive, in inches, measured by the Pinpoint. 24 inches is clearly off the wall.
    // Keep the path clear: by our (unmeasured) numbers the near edge of the hive frame is only
    // about 46 to 51 inches from the wall. A robot that starts against a wall ends with its far
    // edge about 42 inches out (24 plus its 17.75 inch length), so there is not much room to spare.
    // This is where the robot should END UP, after it has finished rolling (see
    // COAST_DISTANCE_INCHES below). The class name and the @Autonomous name both say "24in". If
    // you change this number, change both names too.
    private static final double DRIVE_DISTANCE_INCHES = 24.0;

    // How far the robot keeps rolling after we tell the wheels to stop, in inches. The Auto stops
    // this much BEFORE DRIVE_DISTANCE_INCHES, so the robot rolls the rest of the way.
    // MEASURED 2026-11-23 at DRIVE_POWER 0.30, with no coast allowance: asked for 24 inches, the
    // robot went 27.4 by the Pinpoint (about 27.5 by tape measure),
    // so it rolled about 3.4 inches too far.
    // TODO: measure again with this setting on each robot. Ends long: make this bigger. Ends
    // short: make it smaller.
    private static final double COAST_DISTANCE_INCHES = 3.4;

    // If the robot has not gone DRIVE_DISTANCE_INCHES after this many seconds, stop anyway. This is
    // only a safety net for a Pinpoint that stops counting. 2.0 is a starting guess: at 0.30 power
    // the robot should need about 1.6 seconds for 24 inches.
    // TODO: time a normal run on the robot and set this a bit longer than that.
    private static final double SAFETY_TIMEOUT_SECONDS = 2.0;

    // Used ONLY when the Pinpoint is not READY at START: drive this many seconds instead of a
    // distance. 1.6 is a starting guess (copied from AutoEmergencyLeave).
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

        telemetry.addLine("Ready. Put the robot on its start spot (clear path), then press START.");
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
        // Tell the TeleOp which alliance we are, so it starts on the right one. (A wrong alliance
        // aims at the other hive and does not look like a failure, so this matters.)
        AutoHandoff.save(ALLIANCE);

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
        //    distance from the start is the hypotenuse. A hypotenuse is never negative, so it
        //    measures the drive the same way going forward or backward. (It also still works if
        //    the robot drifts a little sideways.)
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
            // Stop a little EARLY, because the robot rolls COAST_DISTANCE_INCHES more after the
            // wheels stop.
            double stopDistanceInches = DRIVE_DISTANCE_INCHES - COAST_DISTANCE_INCHES;
            boolean reachedDistance = distanceDrivenInches >= stopDistanceInches;
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
        // The alliance is first, because a wrong alliance does not look like a failure.
        telemetry.addData("Alliance", ALLIANCE);
        telemetry.addData("Direction", "FORWARD");
        telemetry.addData("Drive power", "%.2f", DRIVE_POWER);
        telemetry.addData("Drive distance (in)", "%.1f", DRIVE_DISTANCE_INCHES);
        telemetry.addData("Coast allowance (in)", "%.1f", COAST_DISTANCE_INCHES);
    }
}
