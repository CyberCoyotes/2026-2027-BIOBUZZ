package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Set;

/*
 * Battery Subsystem
 * Reads the battery voltage that EACH hub (the Control Hub and the Expansion Hub) sees, and
 * remembers the LOWEST voltage each one has seen. It only READS. It never moves a motor, and it
 * has no gamepad code.
 *
 * WHY IT EXISTS: the robot log from 10-26 showed the Expansion Hub warning "battery too low to
 * run motors and servos" many times during a TeleOp run, while the Control Hub did not. A battery
 * voltage dips for only a moment when a motor starts or stalls, so the warning on the Driver
 * Station comes and goes before anyone can read it. This class keeps the LOWEST number so the dip
 * is still on the screen afterward.
 *
 * HOW TO READ IT (a starting guess, not a rule):
 *   - A charged battery at rest shows somewhere around 12.5 to 13.5 V. A number that is a lot
 *     lower AT REST means a tired battery.
 *   - A big drop for just a moment when a motor starts (like the flywheel spinning up) is normal.
 *   - If ONE hub's lowest number is much lower than the other hub's, look at the power wire and
 *     connector going to that hub before you blame the battery.
 *
 * WHAT IT ANSWERS: "How many volts does each hub see now, and how low did it go?"
 *   getLowestVoltage()  -> the lowest voltage ANY hub has seen since the last reset
 *   isLow()             -> true if ANY hub is under LOW_VOLTAGE_WARNING_VOLTS right now
 *
 * The OpMode must call update() ONCE per loop(). Nothing calls it for us.
 *
 * Robot configuration: NONE. The hubs show up on their own, so there is no name to add. (That is
 * why this class has no hardware name constant like the other subsystems.)
 */
public class BatterySubsystem {

    // ------------------------------------------------------------------
    // Tunable numbers
    // ------------------------------------------------------------------
    // The screen says LOW when a hub drops under this many volts. [CONFIRM] This is a STARTING
    // GUESS, not a number from the rulebook or a spec sheet. Watch what a healthy robot shows
    // while the flywheel spins up, then set this a little under that.
    private static final double LOW_VOLTAGE_WARNING_VOLTS = 11.0;

    // How long to wait between voltage reads. Asking a hub for its voltage is a real message over
    // the USB wire to that hub, so doing it on every loop() would slow the loop down a little.
    // Ten times a second is plenty to catch a dip.
    private static final double READ_INTERVAL_SECONDS = 0.10;

    // ------------------------------------------------------------------
    // The hardware
    // ------------------------------------------------------------------
    // One entry for each hub, in the same order in all three lists below. Empty if the SDK found
    // no voltage sensor (this class then does nothing and the telemetry says so).
    private final ArrayList<VoltageSensor> voltageSensors = new ArrayList<VoltageSensor>();
    private final ArrayList<String> hubNames = new ArrayList<String>();
    private final ArrayList<Double> voltages = new ArrayList<Double>();
    private final ArrayList<Double> lowestVoltages = new ArrayList<Double>();

    // Counts the time since the last read.
    private final ElapsedTime readTimer = new ElapsedTime();

    // ==================================================================
    // SETUP
    // ==================================================================

    /*
     * Finds every hub's voltage sensor and takes a first reading.
     */
    public BatterySubsystem(HardwareMap hardwareMap) {
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensors.add(sensor);
            hubNames.add(getHubName(hardwareMap, sensor));
            voltages.add(Double.valueOf(0.0));
            lowestVoltages.add(Double.valueOf(Double.MAX_VALUE));
        }
        readAllHubs();
    }

    // ==================================================================
    // ACTIONS
    // ==================================================================

    /*
     * Call this ONCE per loop(). It re-reads the hubs, but only every READ_INTERVAL_SECONDS, so
     * calling it every loop is fine.
     */
    public void update() {
        if (readTimer.seconds() >= READ_INTERVAL_SECONDS) {
            readAllHubs();
        }
    }

    /*
     * Forgets the lowest numbers, so the next ones start fresh. Call it at START so the quiet
     * time during INIT does not count.
     */
    public void resetLowestVoltages() {
        for (int i = 0; i < voltages.size(); i++) {
            lowestVoltages.set(i, voltages.get(i));
        }
    }

    // ==================================================================
    // QUESTIONS
    // ==================================================================

    /*
     * The lowest voltage ANY hub has seen since the last reset. Zero if there are no hubs.
     */
    public double getLowestVoltage() {
        if (voltages.isEmpty()) {
            return 0.0;
        }
        double lowest = Double.MAX_VALUE;
        for (int i = 0; i < lowestVoltages.size(); i++) {
            if (lowestVoltages.get(i) < lowest) {
                lowest = lowestVoltages.get(i);
            }
        }
        return lowest;
    }

    /*
     * True if ANY hub is under LOW_VOLTAGE_WARNING_VOLTS right now.
     */
    public boolean isLow() {
        for (int i = 0; i < voltages.size(); i++) {
            if (voltages.get(i) < LOW_VOLTAGE_WARNING_VOLTS) {
                return true;
            }
        }
        return false;
    }

    /*
     * Shows ONE line, so it does not crowd the Driver Station screen. Call it after update().
     * Example:  Battery = Control Hub 12.4 (low 11.8)   Expansion Hub 2 12.3 (low 10.2)
     * The word LOW! appears in front if a hub is under the warning number right now. The words
     * "dipped under" appear if one went under earlier but has recovered.
     */
    public void addTelemetry(Telemetry telemetry) {
        if (voltages.isEmpty()) {
            telemetry.addData("Battery", "no voltage sensor found");
            return;
        }

        String line = "";
        for (int i = 0; i < voltages.size(); i++) {
            line = line + String.format("%s %.1f V (low %.1f)   ",
                    hubNames.get(i), voltages.get(i), lowestVoltages.get(i));
        }

        String warning = "";
        if (isLow()) {
            warning = "LOW! ";
        } else if (getLowestVoltage() < LOW_VOLTAGE_WARNING_VOLTS) {
            warning = "dipped under " + LOW_VOLTAGE_WARNING_VOLTS + " V earlier. ";
        }
        telemetry.addData("Battery", warning + line);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    /*
     * Asks every hub for its voltage, saves it, and lowers the "lowest" number if it went down.
     */
    private void readAllHubs() {
        readTimer.reset();
        for (int i = 0; i < voltageSensors.size(); i++) {
            double volts = voltageSensors.get(i).getVoltage();
            voltages.set(i, volts);
            if (volts < lowestVoltages.get(i)) {
                lowestVoltages.set(i, volts);
            }
        }
    }

    /*
     * The name the robot configuration gives this hub, like "Control Hub" or "Expansion Hub 2".
     * (The sensor's own device name is the same for every hub, so it would not tell them apart.)
     */
    private String getHubName(HardwareMap hardwareMap, VoltageSensor sensor) {
        Set<String> names = hardwareMap.voltageSensor.getNamesOf(sensor);
        if (names.isEmpty()) {
            return "Hub";
        }
        return names.iterator().next();
    }
}
