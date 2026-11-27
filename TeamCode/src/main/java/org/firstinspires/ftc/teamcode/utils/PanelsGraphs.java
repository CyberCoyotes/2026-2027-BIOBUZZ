package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlowerPickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Panels Graphs
 * The ONE place our code talks to Panels (the live dashboard you open in a browser on a laptop
 * that is on the robot's wifi). Every TeleOp, test OpMode and Auto sends its graph numbers
 * through this class. Nothing else imports the Panels library, so if Panels ever causes trouble
 * there is one file to fix. How to open Panels and read the graphs: TeamCode/docs/biobuzz-panels-how-to.md.
 *
 * WHAT IT IS (and is not)
 *   A plain class with NO hardware and NO gamepad (like BiobuzzLog and FiringMonitor, so its name
 *   does not end in "Subsystem"). It only LOOKS at the subsystems through their get...() methods.
 *   It never moves anything and it never changes what the robot does. It also does NOT touch the
 *   Driver Station screen: Panels has its own screen.
 *
 * THE MASTER SWITCH
 *   PANELS_ENABLED below turns Panels graphs on or off for EVERY OpMode at once: the TeleOp, all
 *   the test OpModes and all the Autos. It is ON for now, so every practice run can be graphed
 *   without editing code first. Panels is for practice (a laptop cannot be connected during a
 *   match), so whether it stays on for competition is the open question in issue #138, due Dec 10.
 *   With the switch false, every method below does nothing and costs nothing.
 *
 * HOW AN OPMODE USES IT: three steps per loop(), always in this order
 *   1. panelsGraphs.beginLoop();      the FIRST line of loop().
 *   2. panelsGraphs.addShooter(...)   then any add(...) lines, anywhere in the loop AFTER step 1.
 *   3. panelsGraphs.send();           the LAST line of loop(), after every add.
 *   And once in init():  panelsGraphs = new PanelsGraphs();
 *
 * WHY THERE MUST BE ONLY ONE send() PER LOOP
 *   Panels sends to the browser at most once every 75 ms (about 13 times a second). If a second
 *   class called Panels' update() in the same loop, Panels would THROW AWAY everything that second
 *   class added, every time. So only this class calls update(), and only from send().
 *
 * WHY beginLoop() EXISTS (the "frame")
 *   Most loops are shorter than 75 ms, so Panels would throw most of our numbers away anyway.
 *   beginLoop() asks Panels "will you really send this loop?". If not, the frame stays CLOSED and
 *   every add... method returns at once without even reading the sensors. That keeps the extra
 *   loop time tiny (this matters for Pedro and the turret, see issue #138). isFrameOpen() tells an
 *   OpMode whether this loop counts, in case it wants to skip a slow calculation of its own.
 *
 * "Loop (ms)" is added for you by send() in every OpMode: how long one trip through loop() took.
 *
 * THE RULES FOR GRAPH NAMES (a graph follows a NAME over time)
 *   - Plain numbers only. NO units inside the value: put the units in the name, "Flywheel RPM".
 *   - Yes/no answers go out as 1 (yes) and 0 (no) with "(1=yes)" in the name, so they draw as steps.
 *   - The SAME name every loop. The same thing has the SAME name in every OpMode, so one Panels
 *     layout works for all of them. Use the add...() sets below, and only make a new name for a
 *     number no set has.
 *   - Text (add Text) shows on Panels' Telemetry widget. It is not graphed.
 */
public class PanelsGraphs {

    // ------------------------------------------------------------------
    // THE MASTER SWITCH  [#138]
    // ------------------------------------------------------------------
    // true  = every OpMode sends its graph numbers to Panels (practice). This is the default.
    // false = nothing is sent to Panels, from any OpMode.
    // Issue #138 asks the coach to re-check this before Dec 10 (competition). To turn Panels off
    // for competition, change this ONE word to false and redeploy. Nothing else needs to change.
    private static final boolean PANELS_ENABLED = true;

    // The Panels telemetry. null means "not sending" (the switch is false, or Panels could not start).
    private TelemetryManager panels = null;

    // Is THIS loop one that Panels will really send? beginLoop() decides, send() closes it again.
    private boolean frameOpen = false;

    // Times one trip through loop(), for the "Loop (ms)" graph.
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopMs = 0.0;
    private int loopCount = 0;

    // ==================================================================
    // SETUP
    // ==================================================================

    /*
     * Call this once in init(). If Panels cannot start for any reason, we say so in the Robot
     * Controller log and carry on without it: the robot must never stop because of a graph.
     */
    public PanelsGraphs() {
        if (PANELS_ENABLED) {
            try {
                // PanelsTelemetry is a Kotlin "object", so in Java we reach it through INSTANCE.
                panels = PanelsTelemetry.INSTANCE.getTelemetry();
            } catch (Exception | LinkageError e) {
                // LinkageError covers a Kotlin class that fails while it starts up.
                panels = null;
                BiobuzzLog.warning("PanelsGraphs: Panels could not be started (%s). Graphs are OFF.", e.toString());
            }
        }
        loopTimer.reset();
    }

    // true = the numbers are going to Panels (the switch is on and Panels started).
    public boolean isOn() {
        return panels != null;
    }

    // ==================================================================
    // THE THREE STEPS OF EVERY LOOP
    // ==================================================================

    /*
     * STEP 1: the first line of loop(). Times the loop, and opens this loop's frame only if Panels
     * will really send it (see "WHY beginLoop() EXISTS" above).
     */
    public void beginLoop() {
        lastLoopMs = loopTimer.milliseconds();
        loopTimer.reset();
        loopCount = loopCount + 1;

        if (panels == null) {
            frameOpen = false;
        } else {
            frameOpen = panels.getShouldUpdateLines();
        }
    }

    // true = the add... methods will really add this loop. false = they return at once.
    public boolean isFrameOpen() {
        return frameOpen;
    }

    /*
     * STEP 3: the last line of loop(), after every add. Sends the frame. (This is Panels' own
     * screen, so it does NOT touch the Driver Station's telemetry.)
     */
    public void send() {
        if (!frameOpen) {
            return;
        }
        // The very first loop() only measures the wait between INIT and START, so leave it out.
        if (loopCount > 1) {
            add("Loop (ms)", lastLoopMs);
        }
        panels.update();
        frameOpen = false;
    }

    // ==================================================================
    // STEP 2: ADD ONE NUMBER, YES/NO, OR LINE OF TEXT
    // ==================================================================

    // One number. Units go in the name, not in the number. (NaN and infinity are skipped: they
    // would break the graph.)
    public void add(String name, double value) {
        if (!frameOpen) {
            return;
        }
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return;
        }
        panels.addData(name, round2(value));
    }

    // A yes/no answer as 1 or 0. Put "(1=yes)" in the name so the graph is easy to read.
    public void addFlag(String name, boolean value) {
        if (!frameOpen) {
            return;
        }
        int number = 0;
        if (value) {
            number = 1;
        }
        panels.addData(name, number);
    }

    // A line of words for Panels' Telemetry widget. Not graphed.
    public void addText(String name, String text) {
        if (!frameOpen) {
            return;
        }
        panels.addData(name, text);
    }

    // ==================================================================
    // STEP 2: READY-MADE SETS. One call adds everything a subsystem can graph.
    // ==================================================================

    /*
     * The flywheel and the hood wheel.
     *   includeFlywheelAmps - true also graphs the flywheel's current. That ASKS THE HUB, so it costs
     *   loop time. It is only asked while the wheels run, and only on a frame that will be sent.
     *   Use true in the TeleOp and the shooter tests. Leave it false in Autos.
     */
    public void addShooter(ShooterSubsystem shooterSubsystem, boolean includeFlywheelAmps) {
        if (!frameOpen) {
            return;
        }
        double flywheelRpm = shooterSubsystem.getFlywheelRpm();
        double flywheelTarget = shooterSubsystem.getTargetFlywheelRpm();
        double tolerance = shooterSubsystem.getAtSpeedToleranceRpm();

        add("Flywheel RPM", flywheelRpm);
        add("Flywheel RPM (raw)", shooterSubsystem.getFlywheelRawRpm());
        add("Flywheel target", flywheelTarget);
        // Negative = too slow. The flywheel counts as at speed while the error is between the bands.
        add("Flywheel error (RPM)", flywheelRpm - flywheelTarget);
        add("At-speed band +", tolerance);
        add("At-speed band -", -tolerance);
        if (includeFlywheelAmps && shooterSubsystem.getState() != ShooterSubsystem.ShooterState.IDLE) {
            add("Flywheel amps", shooterSubsystem.getFlywheelCurrentAmps());
        }
        // The hood wheel runs about twice as fast, so it gets its own names (and its own graph).
        add("Hood wheel RPM", shooterSubsystem.getHoodWheelRpm());
        add("Hood wheel target", shooterSubsystem.getTargetHoodWheelRpm());
    }

    // The turret's angle against where it is told to go.
    public void addTurret(TurretSubsystem turretSubsystem) {
        if (!frameOpen) {
            return;
        }
        add("Turret angle (deg)", turretSubsystem.getAngleDegrees());
        add("Turret target (deg)", turretSubsystem.getTargetAngleDegrees());
        addFlag("Turret on target (1=yes)", turretSubsystem.isOnTarget());
    }

    // The Limelight. tx and ty only mean something while the camera sees the hive, so no number without it.
    public void addVision(VisionSubsystem visionSubsystem) {
        if (!frameOpen) {
            return;
        }
        boolean hasTarget = visionSubsystem.hasTarget();
        addFlag("Camera sees hive (1=yes)", hasTarget);
        add("Camera result age (ms)", visionSubsystem.getResultAgeMs());
        if (hasTarget) {
            add("Camera tx (deg)", visionSubsystem.getTxDegrees());
            add("Camera ty (deg)", visionSubsystem.getTyDegrees());
        }
    }

    // The Pinpoint odometry computer: where it thinks the robot is, and how fast it is moving.
    public void addPinpoint(PinpointSubsystem pinpointSubsystem) {
        if (!frameOpen) {
            return;
        }
        // A robot whose configuration has no "pinpoint" has nothing to graph.
        if (!pinpointSubsystem.isAvailable()) {
            return;
        }
        add("Pinpoint X (in)", pinpointSubsystem.getXInches());
        add("Pinpoint Y (in)", pinpointSubsystem.getYInches());
        add("Pinpoint heading (deg)", pinpointSubsystem.getHeadingDegrees());
        add("Pinpoint speed X (in/s)", pinpointSubsystem.getXSpeedInchesPerSecond());
        add("Pinpoint speed Y (in/s)", pinpointSubsystem.getYSpeedInchesPerSecond());
        add("Pinpoint turn speed (deg/s)", pinpointSubsystem.getHeadingSpeedDegreesPerSecond());
    }

    // The Control Hub's built-in gyro.
    public void addImu(ImuSubsystem imuSubsystem) {
        if (!frameOpen) {
            return;
        }
        if (!imuSubsystem.isAvailable()) {
            return;
        }
        add("IMU heading (deg)", imuSubsystem.getHeadingDegrees());
        add("IMU pitch (deg)", imuSubsystem.getPitchDegrees());
        add("IMU roll (deg)", imuSubsystem.getRollDegrees());
    }

    // The intake: the power last sent to the motor (negative = reversing) and how fast it really turns.
    public void addIntake(IntakeSubsystem intakeSubsystem) {
        if (!frameOpen) {
            return;
        }
        add("Intake motor power", intakeSubsystem.getMotorPower());
        add("Intake speed (ticks/s)", intakeSubsystem.getTicksPerSecond());
    }

    // The shooter gate. A servo cannot say where it really is, so this is where we TOLD it to go.
    public void addGate(ShooterServoSubsystem shooterServoSubsystem) {
        if (!frameOpen) {
            return;
        }
        int gateNumber = 0;
        if (shooterServoSubsystem.getState() == ShooterServoSubsystem.GateState.OPENED) {
            gateNumber = 1;
        } else if (shooterServoSubsystem.getState() == ShooterServoSubsystem.GateState.LIFTED) {
            gateNumber = 2;
        }
        add("Gate (0 closed, 1 opened, 2 lifted)", gateNumber);
        add("Gate servo target", shooterServoSubsystem.getTargetPosition());
    }

    // The flower picker bar (two mirrored servos): where each one is told to go.
    public void addFlowerPicker(FlowerPickerSubsystem flowerPickerSubsystem) {
        if (!frameOpen) {
            return;
        }
        add("Flower left servo target", flowerPickerSubsystem.getLeftTargetPosition());
        add("Flower right servo target", flowerPickerSubsystem.getRightTargetPosition());
        addFlag("Flower picker up (1=yes)", flowerPickerSubsystem.isUp());
    }

    // What the driver is asking the drivetrain to do (the sticks, from -1 to 1). The wheel powers
    // themselves are private inside DriveSubsystem, so we graph what was asked for.
    public void addDriveInputs(double forward, double strafe, double turn) {
        if (!frameOpen) {
            return;
        }
        add("Drive forward input", forward);
        add("Drive strafe input", strafe);
        add("Drive turn input", turn);
    }

    // Where Pedro says the robot is (the Pedro Autos pass follower.pose() numbers in). Inches and degrees.
    public void addPose(double xInches, double yInches, double headingDegrees) {
        if (!frameOpen) {
            return;
        }
        add("Pose X (in)", xInches);
        add("Pose Y (in)", yInches);
        add("Pose heading (deg)", headingDegrees);
    }

    // ==================================================================
    // Helper
    // ==================================================================

    // Round to two decimal places, so Panels does not send 1299.8765432.
    private double round2(double value) {
        return Math.round(value * 100.0) / 100.0;
    }
}
