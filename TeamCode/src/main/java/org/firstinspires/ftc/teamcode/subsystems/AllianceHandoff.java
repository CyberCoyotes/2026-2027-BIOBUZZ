package org.firstinspires.ftc.teamcode.subsystems;

/*
 * Alliance Handoff
 * A plain class with NO hardware, no update(), and no gamepad code. It is not a mechanism, so its
 * name does not end in "Subsystem" (the same as ShotTable and HiveBearingMemory). It does one
 * job: let an Auto TELL the TeleOp which alliance it just played, so the TeleOp starts on the
 * right alliance (and so the right field-centric drive direction) without anyone pressing START.
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
 *   An Auto, at START:    AllianceHandoff.save(alliance);
 *   The TeleOp, at INIT:  if (AllianceHandoff.hasFreshAlliance()) { use AllianceHandoff.getAlliance(); }
 *   The operator can still switch the alliance with START on either gamepad, as always.
 */
public class AllianceHandoff {

    // How long a saved alliance stays good, in seconds. A match is an Auto (30 s) and then the
    // TeleOp starts right away, so 300 s (5 minutes) is plenty. The limit is there so a red Auto
    // from this morning cannot make the TeleOp start on RED when we practice an hour later.
    private static final double MAX_AGE_SECONDS = 300.0;

    private static final double NANOSECONDS_PER_SECOND = 1000000000.0;

    // The saved alliance, or null when nothing has been saved (or it was cleared).
    private static VisionSubsystem.Alliance savedAlliance = null;

    // When it was saved, from System.nanoTime() (a clock that only counts forward).
    private static long savedAtNanos = 0;

    // Nobody makes an AllianceHandoff object. Everything here is static, so you just write
    // AllianceHandoff.save(...) and so on.
    private AllianceHandoff() {
    }

    /*
     * An Auto calls this at START with the alliance it is playing.
     */
    public static void save(VisionSubsystem.Alliance alliance) {
        savedAlliance = alliance;
        savedAtNanos = System.nanoTime();
    }

    /*
     * Forget the saved alliance. An Auto that does not know its alliance calls this at START, so
     * an OLD alliance cannot be mistaken for the one that was just played.
     */
    public static void clear() {
        savedAlliance = null;
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
