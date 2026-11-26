package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.RobotLog;

/*
 * Biobuzz Log
 * One place for writing notes to the Robot Controller LOG. The Driver Station screen is erased
 * every loop, so a blink that lasts a moment is gone before anyone reads it. A line written here
 * stays in the log with the time it happened, so you can read it after the match.
 *
 * It has no hardware and no state (like ShotTable), so its name does not end in "Subsystem".
 *
 * HOW TO READ THE LOG after a match:
 *   - With a laptop on the robot's wifi:   adb logcat -d | grep BIOBUZZ
 *   - Or download the Robot Controller log from the Control Hub and search it for BIOBUZZ.
 * Every line starts with the tag BIOBUZZ, so one search finds all of them.
 *
 * warning() is for things that should not happen (a dip in the battery, the Pinpoint dropping
 * out). info() is for the normal "it recovered" follow-up. Both take a format like String.format
 * does: BiobuzzLog.warning("battery %.2f V", volts).
 */
public class BiobuzzLog {

    // The tag that every line of ours carries. Search the log for this word.
    public static final String TAG = "BIOBUZZ";

    public static void warning(String format, Object... values) {
        RobotLog.ww(TAG, format, values);
    }

    public static void info(String format, Object... values) {
        RobotLog.ii(TAG, format, values);
    }
}
