package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

/*
 * Hub Power Subsystem
 * Owns the POWER SENSORS inside the Control Hub and the Expansion Hub. It only READS. It never
 * moves a motor and has no gamepad code. It exists to help us find battery and wiring problems:
 * a worn battery, a loose connector, or a short shows up as a voltage that sags, a current that
 * spikes, or a hub that stops answering.
 *
 * WHAT IT SHOWS, for each hub:
 *   - the battery voltage the hub sees RIGHT NOW, and the LOWEST it has been since START
 *   - the total current the hub is using right now, and the HIGHEST so far
 *   - the I2C bus current (the Pinpoint and other I2C sensors are powered from there; a short or
 *     a bad cable shows up as a big number)
 *   - whether the hub is NOT RESPONDING, and how many times a hub has dropped out
 *
 * HOW TO USE IT: watch the lowest voltage while you drive, spin the shooter and move the turret.
 * Do the same with two different batteries. The battery whose lowest voltage is much lower is the
 * weak one. A hub that says NOT RESPONDING usually lost power for a moment, so check the battery
 * connector, the power switch and that hub's power wires.
 *
 * It does not need a name in the robot configuration. It finds every hub by itself.
 *
 * THE COST: asking a hub for its voltage and current is a trip over the hub's cable, so it takes
 * a little time. update() only does it every READ_INTERVAL_SECONDS, not every loop. If the
 * "Loop time" on the screen gets worse after this was added, make that number bigger.
 *
 * The OpMode must call update() ONCE per loop(). Nothing calls it for us.
 */
public class HubPowerSubsystem {

    // ------------------------------------------------------------------
    // Settings
    // ------------------------------------------------------------------
    // How often (in seconds) we read the hubs. 0.25 is four times a second, fast enough to catch a
    // dip that lasts a moment. A bigger number means less loop time used.
    private static final double READ_INTERVAL_SECONDS = 0.25;

    // The Control Hub always has this address. Any other hub is an Expansion Hub. We use it only
    // to print the right name.
    private static final int CONTROL_HUB_ADDRESS = 173;

    // The screen adds "LOW" when a hub sees less than this many volts. A good 12 volt battery is
    // about 13 volts when full and rested. [CONFIRM] This is a starting guess. Watch the numbers
    // from a good battery and from a worn one, and set it between them.
    private static final double LOW_VOLTAGE_WARNING_VOLTS = 10.5;

    // A reading at or below this many volts is not a real reading (the hub did not answer), so we
    // do not let it count as the "lowest".
    private static final double NOT_A_REAL_VOLTAGE = 0.1;

    // ------------------------------------------------------------------
    // The hubs and what we last read from each. The arrays have one spot per hub, in the same order
    // as the list.
    // ------------------------------------------------------------------
    private final List<LynxModule> hubs;
    private final double[] voltageVolts;
    private final double[] lowestVoltageVolts;
    private final double[] currentAmps;
    private final double[] highestCurrentAmps;
    private final double[] i2cCurrentAmps;
    private final boolean[] hubNotResponding;
    private final boolean[] hubWasNotResponding;
    private final boolean[] haveReading;

    // How many times any hub has gone from answering to NOT answering.
    private int dropoutCount = 0;

    // Times how long since the last read, so we only read every READ_INTERVAL_SECONDS.
    private final ElapsedTime readTimer = new ElapsedTime();
    private boolean hasReadOnce = false;

    // ==================================================================
    // SETUP
    // ==================================================================

    public HubPowerSubsystem(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        int hubCount = hubs.size();
        voltageVolts = new double[hubCount];
        lowestVoltageVolts = new double[hubCount];
        currentAmps = new double[hubCount];
        highestCurrentAmps = new double[hubCount];
        i2cCurrentAmps = new double[hubCount];
        hubNotResponding = new boolean[hubCount];
        hubWasNotResponding = new boolean[hubCount];
        haveReading = new boolean[hubCount];
        resetLowestAndHighest();
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the subsystem what to do
    // ==================================================================

    /*
     * Call this ONCE per loop(). Most calls do nothing, because it only reads the hubs every
     * READ_INTERVAL_SECONDS.
     */
    public void update() {
        if (hasReadOnce && readTimer.seconds() < READ_INTERVAL_SECONDS) {
            return;
        }
        readTimer.reset();
        hasReadOnce = true;

        for (int i = 0; i < hubs.size(); i++) {
            LynxModule hub = hubs.get(i);

            // A hub that is not answering cannot give a reading, so skip it. Count each time one
            // STARTS not answering, so one long dropout counts once.
            boolean notResponding = hub.isNotResponding();
            hubNotResponding[i] = notResponding;
            if (notResponding && !hubWasNotResponding[i]) {
                dropoutCount = dropoutCount + 1;
            }
            hubWasNotResponding[i] = notResponding;
            if (notResponding) {
                continue;
            }

            double voltage = hub.getInputVoltage(VoltageUnit.VOLTS);
            double current = hub.getCurrent(CurrentUnit.AMPS);
            double i2cCurrent = hub.getI2cBusCurrent(CurrentUnit.AMPS);

            if (voltage <= NOT_A_REAL_VOLTAGE) {
                continue;
            }

            voltageVolts[i] = voltage;
            currentAmps[i] = current;
            i2cCurrentAmps[i] = i2cCurrent;
            haveReading[i] = true;

            if (voltage < lowestVoltageVolts[i]) {
                lowestVoltageVolts[i] = voltage;
            }
            if (current > highestCurrentAmps[i]) {
                highestCurrentAmps[i] = current;
            }
        }
    }

    /*
     * Forgets the lowest voltage and the highest current so far, so the numbers start fresh.
     * Call it at START, so what you read afterward is only about the run.
     */
    public void resetLowestAndHighest() {
        for (int i = 0; i < hubs.size(); i++) {
            lowestVoltageVolts[i] = Double.MAX_VALUE;
            highestCurrentAmps[i] = 0.0;
        }
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks about the power
    // ==================================================================

    /*
     * The lowest battery voltage any hub has seen since the last reset, or 0.0 if there has been
     * no reading yet.
     */
    public double getLowestVoltageVolts() {
        double lowest = Double.MAX_VALUE;
        for (int i = 0; i < hubs.size(); i++) {
            if (haveReading[i] && lowestVoltageVolts[i] < lowest) {
                lowest = lowestVoltageVolts[i];
            }
        }
        if (lowest == Double.MAX_VALUE) {
            return 0.0;
        }
        return lowest;
    }

    /*
     * How many times a hub has stopped answering since we were created.
     */
    public int getDropoutCount() {
        return dropoutCount;
    }

    /*
     * Shows each hub's voltage and current. Call it after update(). It prints its own heading, so
     * the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- HUB POWER ---");

        if (hubs.isEmpty()) {
            telemetry.addData("Hubs", "none found");
            return;
        }

        for (int i = 0; i < hubs.size(); i++) {
            String name = getHubName(hubs.get(i));
            if (hubNotResponding[i]) {
                telemetry.addData(name, "NOT RESPONDING (it may have lost power)");
            } else if (!haveReading[i]) {
                telemetry.addData(name, "no reading yet");
            } else {
                String lowText = "";
                if (voltageVolts[i] < LOW_VOLTAGE_WARNING_VOLTS) {
                    lowText = "   LOW BATTERY";
                }
                telemetry.addData(name, "%.1f V (lowest %.1f)   %.1f A (highest %.1f)   I2C %.2f A%s",
                        voltageVolts[i], lowestVoltageVolts[i],
                        currentAmps[i], highestCurrentAmps[i],
                        i2cCurrentAmps[i], lowText);
            }
        }
        telemetry.addData("Hub dropouts", "%d", dropoutCount);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    private String getHubName(LynxModule hub) {
        int address = hub.getModuleAddress();
        if (address == CONTROL_HUB_ADDRESS) {
            return "Control Hub";
        }
        return "Expansion Hub " + address;
    }
}
