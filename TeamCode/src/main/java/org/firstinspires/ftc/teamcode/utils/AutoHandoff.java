package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Auto Handoff
 * A plain class with NO hardware, no update(), and no gamepad code. It is not a mechanism, so its
 * name does not end in "Subsystem" (the same as ShotTable and HiveBearingMemory). It does one
 * job: let an Auto TELL the TeleOp two things, so the TeleOp starts with the right field-centric
 * drive direction without anyone pressing a button:
 *   1. which ALLIANCE it just played (so the TeleOp starts on the same alliance), and
 *   2. which way the robot FACES at the end of the Auto (its heading on the field).
 *
 * WHY THE HEADING MATTERS
 *   The field-centric drive needs to know which way the robot faces on the field when the TeleOp
 *   starts. Without the heading it has to guess "the same way the Auto started", and that is wrong
 *   after an Auto that turns the robot (the 3 Volley Autos). With the heading the guess is not
 *   needed. The heading is in Pedro's field frame (0 degrees faces the right wall, 90 faces up
 *   the field, turning left is positive), the same frame the TeleOp's drive uses.
 *
 * HOW IT WORKS (and why the fields say "static")
 *   Every OpMode is a separate object. When the Auto ends and the TeleOp starts, the TeleOp is a
 *   brand new object and cannot see the Auto's variables. A "static" variable belongs to the CLASS,
 *   not to an object, and the Robot Controller app keeps classes loaded the whole time it is
 *   running. So a static variable written by the Auto can still be read by the TeleOp.
 *
 *   This is the only static state in our code. We keep it in this one small class, so there is
 *   exactly one place to look when the TeleOp starts on an alliance you did not expect.
 *
 * WHEN IT DOES NOT WORK (the TeleOp then falls back to BLUE, like before)
 *   - The Robot Controller app was restarted, or the Control Hub was power cycled, between the
 *     Auto and the TeleOp. The statics start empty again.
 *   - The Auto was never started (INIT only). Autos save the alliance at START, not at INIT.
 *   - The Auto does not know the alliance (AutoEmergencyLeave). It calls clear() instead.
 *   - Too much time has passed. See MAX_AGE_SECONDS.
 *
 * HOW THE OPMODES USE IT
 *   An Auto, at START:    AutoHandoff.save(alliance);
 *   An Auto, at STOP:     AutoHandoff.saveHeading(final heading in degrees)   (only if it started)
 *   The TeleOp, at INIT:  if (AutoHandoff.hasFreshAlliance()) { use AutoHandoff.getAlliance(); }
 *                         if (AutoHandoff.hasFreshHeading())  { use AutoHandoff.getHeadingDegrees(); }
 *   The TeleOp, at START: AutoHandoff.clearHeading();   (see below)
 *   The operator can still switch the alliance with START on either gamepad, as always. The
 *   driver can still re-aim with D-pad Left, as always.
 *
 * WHY THE TELEOP "SPENDS" THE HEADING AT START
 *   The heading is only true until the robot moves. Once the TeleOp starts driving, a heading
 *   from the Auto is out of date. If the TeleOp is stopped and started again, it must NOT use it.
 *   (Pressing INIT again BEFORE START is fine: the robot has not moved, so it is still true.)
 */
public class AutoHandoff {

    // How long a saved alliance stays good, in seconds. A match is an Auto (30 s) and then the
    // TeleOp starts right away, so 300 s (5 minutes) is plenty. The limit is there so a red Auto
    // from this morning cannot make the TeleOp start on RED when we practice an hour later.
    private static final double MAX_AGE_SECONDS = 300.0;

    private static final double NANOSECONDS_PER_SECOND = 1000000000.0;

    // The saved alliance, or null when nothing has been saved (or it was cleared).
    private static VisionSubsystem.Alliance savedAlliance = null;

    // The robot's heading at the end of the Auto, in degrees on Pedro's field. It only counts when
    // headingSaved is true.
    private static double savedHeadingDegrees = 0.0;
    private static boolean headingSaved = false;

    // When it was saved, from System.nanoTime() (a clock that only counts forward).
    private static long savedAtNanos = 0;

    // Nobody makes an AutoHandoff object. Everything here is static, so you just write
    // AutoHandoff.save(...) and so on.
    private AutoHandoff() {
    }

    /*
     * An Auto calls this at START with the alliance it is playing.
     */
    public static void save(VisionSubsystem.Alliance alliance) {
        savedAlliance = alliance;
        savedAtNanos = System.nanoTime();
        // A NEW Auto is starting, so a heading left over from an earlier one is out of date. The
        // new Auto saves its own heading when it stops.
        headingSaved = false;
    }

    /*
     * An Auto calls this at STOP with the heading the robot ended on, in DEGREES on Pedro's field
     * (Math.toDegrees(follower.pose().heading())). Only call it if the Auto actually started.
     * It does nothing unless save() was called first, so a heading never gets saved on its own.
     */
    public static void saveHeading(double fieldHeadingDegrees) {
        if (savedAlliance == null) {
            return;
        }
        savedHeadingDegrees = fieldHeadingDegrees;
        headingSaved = true;
        // Count the age from the END of the Auto, which is when the TeleOp is about to start.
        savedAtNanos = System.nanoTime();
    }

    /*
     * Forget the saved alliance. An Auto that does not know its alliance calls this at START, so
     * an OLD alliance cannot be mistaken for the one that was just played.
     */
    public static void clear() {
        savedAlliance = null;
        headingSaved = false;
    }

    /*
     * Forget only the heading. The TeleOp calls this at START, because once it starts driving the
     * heading from the Auto is no longer true. (The alliance stays: it is still true.)
     */
    public static void clearHeading() {
        headingSaved = false;
    }

    /*
     * True when an Auto saved an alliance recently enough to trust it.
     */
    public static boolean hasFreshAlliance() {
        if (savedAlliance == null) {
            return false;
        }
        double ageSeconds = (System.nanoTime() - savedAtNanos) / NANOSECONDS_PER_SECOND;
        return ageSeconds <= MAX_AGE_SECONDS;
    }

    /*
     * True when an Auto saved its ending heading recently enough to trust it.
     */
    public static boolean hasFreshHeading() {
        return headingSaved && hasFreshAlliance();
    }

    /*
     * The heading the robot ended the Auto on, in degrees on Pedro's field. Ask hasFreshHeading()
     * first. If there is no fresh one, this gives 0.0, which means nothing: do not use it.
     */
    public static double getHeadingDegrees() {
        if (!hasFreshHeading()) {
            return 0.0;
        }
        return savedHeadingDegrees;
    }

    /*
     * The saved alliance. Ask hasFreshAlliance() first. If there is no fresh one, this gives BLUE,
     * the same default the TeleOp has always had.
     */
    public static VisionSubsystem.Alliance getAlliance() {
        if (!hasFreshAlliance()) {
            return VisionSubsystem.Alliance.BLUE;
        }
        return savedAlliance;
    }
}
