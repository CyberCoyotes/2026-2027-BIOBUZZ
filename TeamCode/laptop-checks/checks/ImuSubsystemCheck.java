import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;

public class ImuSubsystemCheck {
    static double fakeYaw = 0, fakePitch = 0, fakeRoll = 0;
    static int resetCalls = 0, readCalls = 0;
    static int failures = 0;
    static void check(String what, boolean ok, String detail) { if (!ok) failures++; System.out.println((ok ? "PASS " : "FAIL ") + what + "   " + detail); }

    static ImuSubsystem make(IMU imu, boolean initialized) throws Exception {
        Field uf = sun.misc.Unsafe.class.getDeclaredField("theUnsafe"); uf.setAccessible(true);
        sun.misc.Unsafe unsafe = (sun.misc.Unsafe) uf.get(null);
        ImuSubsystem s = (ImuSubsystem) unsafe.allocateInstance(ImuSubsystem.class);   // skips the constructor (needs a HardwareMap)
        Field a = ImuSubsystem.class.getDeclaredField("imu"); a.setAccessible(true); a.set(s, imu);
        Field b = ImuSubsystem.class.getDeclaredField("initialized"); b.setAccessible(true); b.set(s, initialized);
        return s;
    }
    public static void main(String[] args) throws Exception {
        IMU fake = (IMU) Proxy.newProxyInstance(IMU.class.getClassLoader(), new Class[]{IMU.class}, (proxy, m, a) -> {
            if (m.getName().equals("getRobotYawPitchRollAngles")) { readCalls++; return new YawPitchRollAngles(AngleUnit.DEGREES, fakeYaw, fakePitch, fakeRoll, 0L); }
            if (m.getName().equals("resetYaw")) { resetCalls++; return null; }
            return null;
        });

        ImuSubsystem imu = make(fake, true);
        check("available when found and started", imu.isAvailable(), "");
        fakeYaw = 90.0; fakePitch = 2.0; fakeRoll = -1.5;
        imu.update();
        check("heading comes from yaw (INVERT_HEADING is false)", Math.abs(imu.getHeadingDegrees() - 90.0) < 1e-9, "heading = " + imu.getHeadingDegrees());
        check("pitch and roll are passed through", imu.getPitchDegrees() == 2.0 && imu.getRollDegrees() == -1.5, "pitch " + imu.getPitchDegrees() + " roll " + imu.getRollDegrees());
        fakeYaw = -179.0; imu.update();
        check("negative headings (turning right) stay negative", Math.abs(imu.getHeadingDegrees() + 179.0) < 1e-9, "heading = " + imu.getHeadingDegrees());
        imu.resetHeading();
        check("resetHeading() tells the IMU and zeroes our copy at once", resetCalls == 1 && imu.getHeadingDegrees() == 0.0, "resetYaw calls " + resetCalls + ", heading " + imu.getHeadingDegrees());

        int readsBefore = readCalls;
        ImuSubsystem missing = make(null, false);
        missing.update(); missing.resetHeading();
        check("no IMU in the config: not available, no crash, heading 0", !missing.isAvailable() && missing.getHeadingDegrees() == 0.0 && readCalls == readsBefore, "");

        ImuSubsystem notStarted = make(fake, false);
        notStarted.update();
        check("IMU found but did not start: not available, never read", !notStarted.isAvailable() && readCalls == readsBefore, "reads " + (readCalls - readsBefore));

        System.out.println(failures == 0 ? "\nALL PASSED" : "\n" + failures + " FAILED");
        System.exit(failures == 0 ? 0 : 1);
    }
}
