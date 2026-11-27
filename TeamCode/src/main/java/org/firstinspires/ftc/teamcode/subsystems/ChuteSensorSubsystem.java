package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Chute Sensor Subsystem (issue #144, Phase 1)
 * Owns the REV Color Sensor V3 near the top of the chute, below the turret. Only this class touches
 * it. OpModes call update() once per loop, then ask it questions.
 *
 * WHAT IT DOES
 *   Phase 1: it READS. It tells you how far away the nearest thing is (the Pollen, we hope) and
 *   what color it is.
 *   Phase 2 (issue #144): it also gives ONE yes / no answer, hasPollenAtTop(): "is a Pollen sitting
 *   at the top of the chute right now?" It uses the DISTANCE only. The color is still just a
 *   reading. It still moves nothing: it does not touch the gate or the intake (that is Phase 3).
 *
 * HOW hasPollenAtTop() DECIDES (Phase 2)
 *   Two distances with a GAP between them, and a short wait:
 *     - At or below POLLEN_AT_TOP_ENTER_CM (3.0 cm): a Pollen is STAGED. The answer turns TRUE.
 *     - At or above POLLEN_AT_TOP_EXIT_CM (3.8 cm), or nothing in range: the top spot is CLEAR.
 *       The answer turns FALSE.
 *     - In between (3.0 to 3.8 cm): the answer does NOT change. It keeps what it said last.
 *   The gap is on purpose. A ball that wobbles or turns would otherwise flip the answer on and off.
 *   The reading must also STAY past the number for POLLEN_AT_TOP_DEBOUNCE_SECONDS before the
 *   answer flips, so a ball rolling past (or a hole turning toward the sensor) for a moment does
 *   not count.
 *   WHEN IN DOUBT THE ANSWER IS FALSE. No sensor, a device that cannot measure distance, "out of
 *   range" and a bad (NaN) reading all count as CLEAR. Later (Phase 3) this answer may lift a ball
 *   into the spinning wheels, so a wrong "false" (the operator uses Y) is much safer than a wrong
 *   "true".
 *   The sensor only sees the TOP SPOT of the chute, not the whole chute. So "false" means "no
 *   Pollen at the top", NOT "the chute is empty".
 *
 * ONE SENSOR, TWO JOBS
 *   The REV Color Sensor V3 is a color sensor AND an infrared distance sensor in one body.
 *     - DISTANCE: the sensor shines infrared light and measures how much comes back. A close, shiny
 *       or light-colored object sends back a lot (reads near). Nothing there reads far. The SDK
 *       sample says this is only useful at very small distances, and it is sensitive to the room
 *       light and to how much light the object reflects. The Pollen has HOLES, so the reading may
 *       change as the ball turns. That is why we measure before we trust it.
 *     - COLOR: red, green and blue amounts (0.0 to 1.0), and the hue (the color's place on the
 *       color wheel, in degrees: yellow is about 40 to 70). Hue only means something when there is
 *       enough light, so we also report getLightLevel().
 *   Each read is one trip over the I2C wires. The color read is the slower one, so the test OpMode
 *   can turn it off (setColorReadingOn) to show what it costs.
 *
 * ONLY SOME ROBOTS HAVE THE SENSOR
 *   Right now only the 22091 robot has it. This class is shared by every robot, so a missing
 *   sensor must NOT crash the OpMode. We use tryGet (like PinpointSubsystem): if "chute_sensor" is
 *   not in the robot configuration, isConnected() is false, update() does nothing, and the screen
 *   says NOT FOUND.
 *
 * STATE vs STATUS: this class has no state (it never tells anything to move). Everything it
 * reports is STATUS: what the sensor reports.
 *
 * Robot configuration (on the Driver Station):
 *   I2C device name: chute_sensor
 *   Device type: REV Color Sensor V3, plugged into a Control Hub I2C bus. The 22091 robot uses
 *   bus 1 (the Pinpoint is on bus 3, and bus 0 is the Hub's own IMU).
 */
public class ChuteSensorSubsystem {

    // ------------------------------------------------------------------
    // Hardware name. This MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    private static final String SENSOR_NAME = "chute_sensor";

    // ------------------------------------------------------------------
    // Sensor settings
    // ------------------------------------------------------------------
    // Gain is a multiplier for the COLOR numbers only (it does not change the distance). Higher gain
    // makes dim scenes read brighter. The SDK sample starts at 2.
    // TODO: starting guess. Look at the color numbers with a Pollen at the top of the chute in
    // ChuteSensorAlpha. If they are all near 0, raise it. If they are all stuck near 1.0, lower it.
    private static final float COLOR_GAIN = 2.0f;

    // Any distance FARTHER than this is treated as "nothing there". The sensor can report a huge
    // number (or nothing useful) when nothing is close, and that must not wreck the min/max or the
    // graph. 100 cm is one meter: far past anything the chute can hold, and far past where this
    // infrared sensor works.
    private static final double OUT_OF_RANGE_CM = 100.0;

    // ------------------------------------------------------------------
    // "Pollen at the top" settings (Phase 2, issue #144)
    // ------------------------------------------------------------------
    // How hasPollenAtTop() uses these is explained at the top of this file. ENTER must stay SMALLER
    // than EXIT: the gap between them is what stops the answer from flickering.
    // TODO (issue #144): 3.0 and 3.8 are the coach's hand readings (a staged Pollen reads about
    // 2.0 +/- 0.5 cm, and an empty chute reads 3.8 to 4.0 cm). Check them with a hole facing the
    // sensor, a turning ball and balls feeding past. 3.8 sits on the edge of the empty reading: if
    // the answer ever stays TRUE after the ball is gone, try a smaller EXIT such as 3.5.
    private static final double POLLEN_AT_TOP_ENTER_CM = 3.0;
    private static final double POLLEN_AT_TOP_EXIT_CM = 3.8;

    // How long a reading must STAY past ENTER (or EXIT) before the answer flips.
    // TODO (issue #144): starting guess. Raise it if balls feeding past make the answer blip TRUE.
    private static final double POLLEN_AT_TOP_DEBOUNCE_SECONDS = 0.10;

    // ------------------------------------------------------------------
    // Hardware. Only this class touches it.
    // ------------------------------------------------------------------
    // The same physical sensor, seen two ways. The REV Color Sensor V3 is BOTH a color sensor and a
    // distance sensor, so it is the same object. colorSensor is null if the config has no
    // "chute_sensor". distanceSensor is null if that device cannot measure distance.
    private NormalizedColorSensor colorSensor = null;
    private DistanceSensor distanceSensor = null;

    // ------------------------------------------------------------------
    // STATUS: what the sensor reports (saved by update())
    // ------------------------------------------------------------------
    private double distanceCm = OUT_OF_RANGE_CM;
    private boolean distanceInRange = false;

    // The Phase 2 answer (see hasPollenAtTop()). It starts FALSE: no Pollen until the sensor says so.
    private boolean pollenAtTop = false;

    // Debounce. readingAsksForFlip stays true while the readings keep asking for the answer to flip,
    // and flipTimer says for how long. A reading in the gap (or one that agrees with the answer)
    // sets it back to false, so the wait starts over.
    private boolean readingAsksForFlip = false;
    private final ElapsedTime flipTimer = new ElapsedTime();

    private double red = 0.0;
    private double green = 0.0;
    private double blue = 0.0;
    private double lightLevel = 0.0;   // the total brightness the color sensor sees (0.0 to 1.0)
    private double hueDegrees = 0.0;

    // true = update() also reads the color. false = it reads only the distance (faster).
    private boolean colorReadingOn = true;

    // How long the last update() spent talking to the sensor, in milliseconds.
    private double readMillis = 0.0;
    private final ElapsedTime readTimer = new ElapsedTime();

    // Space for Color.colorToHSV to put its answer: [0] hue, [1] saturation, [2] value.
    private final float[] hsvValues = new float[3];

    // ==================================================================
    // SETUP
    // ==================================================================

    /*
     * Looks for the sensor in the robot configuration. A missing sensor is not an error here (see
     * "ONLY SOME ROBOTS HAVE THE SENSOR" at the top).
     */
    public ChuteSensorSubsystem(HardwareMap hardwareMap) {
        // tryGet gives back null instead of stopping the OpMode when the name is not in the robot
        // configuration.
        colorSensor = hardwareMap.tryGet(NormalizedColorSensor.class, SENSOR_NAME);
        if (colorSensor == null) {
            return;
        }

        colorSensor.setGain(COLOR_GAIN);

        // The color sensor object can also measure distance, if it is the right kind of sensor.
        if (colorSensor instanceof DistanceSensor) {
            distanceSensor = (DistanceSensor) colorSensor;
        }
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the subsystem what to do
    // ==================================================================

    /*
     * Turns the color reading on or off. Off means update() reads only the distance, which is
     * quicker. The test OpMode uses this to show what the color read costs. It takes effect on the
     * next update().
     */
    public void setColorReadingOn(boolean on) {
        colorReadingOn = on;
    }

    /*
     * Call this ONCE per loop(). It reads the sensor and saves the newest answers. Every getter
     * below returns what this saved, so asking many times in one loop costs nothing extra.
     */
    public void update() {
        if (colorSensor == null) {
            return;
        }

        readTimer.reset();

        if (distanceSensor != null) {
            double rawCm = distanceSensor.getDistance(DistanceUnit.CM);
            if (Double.isNaN(rawCm) || rawCm > OUT_OF_RANGE_CM) {
                distanceInRange = false;
                distanceCm = OUT_OF_RANGE_CM;
            } else {
                distanceInRange = true;
                distanceCm = rawCm;
            }
        }

        // Phase 2: use the distance we just saved to work out the "Pollen at the top" answer.
        updatePollenAtTop();

        if (colorReadingOn) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            red = colors.red;
            green = colors.green;
            blue = colors.blue;
            lightLevel = colors.alpha;

            // Hue is the color's angle on the color wheel (0 to 360 degrees).
            Color.colorToHSV(colors.toColor(), hsvValues);
            hueDegrees = hsvValues[0];
        }

        readMillis = readTimer.milliseconds();
    }

    /*
     * Works out the "Pollen at the top" answer from the distance update() just saved. Called once
     * per update(). The rule is explained at the top of this file.
     */
    private void updatePollenAtTop() {
        // No distance sensor means no Pollen answer. (With no sensor at all, update() never gets
        // here, and the answer just stays FALSE.)
        if (distanceSensor == null) {
            pollenAtTop = false;
            readingAsksForFlip = false;
            return;
        }

        // "Out of range" and a bad (NaN) reading were both saved as OUT_OF_RANGE_CM (100) in
        // update(). That is far past EXIT, so they already count as CLEAR here.
        boolean readingSaysStaged = distanceCm <= POLLEN_AT_TOP_ENTER_CM;
        boolean readingSaysClear = distanceCm >= POLLEN_AT_TOP_EXIT_CM;

        // Does this reading ask the answer to FLIP? Only a STAGED reading can turn it TRUE, and
        // only a CLEAR reading can turn it FALSE. A reading in the gap asks for nothing.
        boolean thisReadingAsksForFlip;
        if (pollenAtTop) {
            thisReadingAsksForFlip = readingSaysClear;
        } else {
            thisReadingAsksForFlip = readingSaysStaged;
        }

        if (!thisReadingAsksForFlip) {
            // The reading is in the gap, or it agrees with the answer. The wait starts over.
            readingAsksForFlip = false;
            return;
        }

        if (!readingAsksForFlip) {
            // The first reading that asks for a flip: start the timer.
            readingAsksForFlip = true;
            flipTimer.reset();
        }

        // The readings have asked for long enough, so flip the answer.
        if (flipTimer.seconds() >= POLLEN_AT_TOP_DEBOUNCE_SECONDS) {
            pollenAtTop = !pollenAtTop;
            readingAsksForFlip = false;
        }
    }

    /*
     * The words for which band the newest distance is in, for the screen: STAGED (at or below
     * ENTER), CLEAR (at or above EXIT, or nothing in range) or BETWEEN (in the gap).
     */
    private String getBandText() {
        if (distanceCm <= POLLEN_AT_TOP_ENTER_CM) {
            return "STAGED";
        }
        if (distanceCm >= POLLEN_AT_TOP_EXIT_CM) {
            return "CLEAR";
        }
        return "BETWEEN";
    }

    // ==================================================================
    // QUESTIONS: what the sensor says (STATUS)
    // ==================================================================

    // true = a Pollen is sitting at the top of the chute (Phase 2: see the top of this file for how
    // it decides). It is false when there is no sensor, and false when unsure. false means "no
    // Pollen at the TOP", not "the chute is empty": the sensor only sees the top spot.
    public boolean hasPollenAtTop() {
        return pollenAtTop;
    }

    // The two distances the answer uses, so a test OpMode can show them without copying the numbers.
    public double getPollenAtTopEnterCm() {
        return POLLEN_AT_TOP_ENTER_CM;
    }

    public double getPollenAtTopExitCm() {
        return POLLEN_AT_TOP_EXIT_CM;
    }

    // true = "chute_sensor" is in the robot configuration. false = NOT FOUND (every other
    // question below then returns its "nothing" answer).
    public boolean isConnected() {
        return colorSensor != null;
    }

    // Distance to the nearest thing, in centimeters. When nothing is in range this is
    // OUT_OF_RANGE_CM (100), so check isDistanceInRange() before trusting it as a real reading.
    public double getDistanceCm() {
        return distanceCm;
    }

    // true = the sensor measured a real distance. false = nothing close enough (or no sensor).
    public boolean isDistanceInRange() {
        return distanceInRange;
    }

    // The color amounts, 0.0 to 1.0. These stay at their last values while the color reading is off.
    public double getRed() {
        return red;
    }

    public double getGreen() {
        return green;
    }

    public double getBlue() {
        return blue;
    }

    // The total light the color sensor sees (0.0 to 1.0). Low means dim, so the hue is not reliable.
    public double getLightLevel() {
        return lightLevel;
    }

    // The color's angle on the color wheel, 0 to 360 degrees. Yellow is about 40 to 70.
    public double getHueDegrees() {
        return hueDegrees;
    }

    public boolean isColorReadingOn() {
        return colorReadingOn;
    }

    // How long the last update() spent on the sensor, in milliseconds. This is the cost of the I2C
    // reads, which come out of every loop.
    public double getReadMillis() {
        return readMillis;
    }

    // ==================================================================
    // TELEMETRY
    // ==================================================================

    /*
     * Shows the sensor's readings. Call it after update(). It prints its own heading, so the OpMode
     * has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- CHUTE SENSOR ---");

        if (colorSensor == null) {
            telemetry.addData("Sensor", "NOT FOUND. Add a REV Color Sensor V3 named \"%s\" to the robot config", SENSOR_NAME);
            return;
        }

        telemetry.addData("Sensor", "FOUND (%s)", SENSOR_NAME);

        if (distanceSensor == null) {
            telemetry.addData("Distance", "this device cannot measure distance. Configure it as a REV Color Sensor V3");
        } else if (distanceInRange) {
            telemetry.addData("Distance", "%.1f cm", distanceCm);
        } else {
            telemetry.addData("Distance", "OUT OF RANGE (nothing close)");
        }

        // Phase 2: the answer, and which band the raw reading is in right now.
        if (distanceSensor != null) {
            String pollenText = "no";
            if (pollenAtTop) {
                pollenText = "YES";
            }
            telemetry.addData("Pollen at top", "%s   (staged at or below %.1f cm, clear at or above %.1f cm)",
                    pollenText, POLLEN_AT_TOP_ENTER_CM, POLLEN_AT_TOP_EXIT_CM);
            telemetry.addData("Reading is in the", "%s band", getBandText());
        }

        if (colorReadingOn) {
            telemetry.addData("Color", "R %.2f  G %.2f  B %.2f   Hue %.0f deg   Light %.2f",
                    red, green, blue, hueDegrees, lightLevel);
        } else {
            telemetry.addData("Color", "reading is OFF");
        }

        telemetry.addData("Sensor read time", "%.1f ms", readMillis);
    }
}
