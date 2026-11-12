import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveAccelerationCheck {
    static double[] lastPower = new double[4];   // what each fake motor was last told
    static DriveSubsystem drive;
    static int failures = 0;

    static DcMotor fakeMotor(final int slot) {
        return (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(), new Class[]{DcMotor.class},
            (proxy, method, args) -> {
                if (method.getName().equals("setPower")) { lastPower[slot] = (Double) args[0]; }
                return null;
            });
    }
    static void set(String name, Object value) throws Exception {
        Field f = DriveSubsystem.class.getDeclaredField(name); f.setAccessible(true); f.set(drive, value);
    }
    static void fresh() throws Exception {
        Field uf = sun.misc.Unsafe.class.getDeclaredField("theUnsafe"); uf.setAccessible(true);
        sun.misc.Unsafe unsafe = (sun.misc.Unsafe) uf.get(null);
        drive = (DriveSubsystem) unsafe.allocateInstance(DriveSubsystem.class);   // skips the constructor (needs a HardwareMap)
        set("leftFrontMotor", fakeMotor(0)); set("rightFrontMotor", fakeMotor(1));
        set("leftRearMotor", fakeMotor(2));  set("rightRearMotor", fakeMotor(3));
        set("powerScale", 1.0); set("accelerationTimer", new ElapsedTime());
        java.util.Arrays.fill(lastPower, 0.0);
    }
    static void loop20ms(double f, double s, double t, int times) throws Exception {
        for (int i = 0; i < times; i++) { Thread.sleep(20); drive.drive(f, s, t); }
    }
    static void check(String what, boolean ok, String detail) {
        if (!ok) failures++;
        System.out.println((ok ? "PASS " : "FAIL ") + what + "   " + detail);
    }

    public static void main(String[] args) throws Exception {
        Method ramp = DriveSubsystem.class.getDeclaredMethod("rampToward", double.class, double.class, double.class);
        ramp.setAccessible(true);
        fresh();
        // --- the pure rule, exact numbers (biggestStep = 0.1) ---
        double[][] table = {
            // current, wanted, expected
            { 0.0,  1.0,  0.1 },   // from rest: one step
            { 0.5,  1.0,  0.6 },   // speeding up: one step
            { 0.95, 1.0,  1.0 },   // never past what is wanted
            { 1.0,  0.0,  0.0 },   // stopping is instant
            { 1.0,  0.5,  0.5 },   // slowing is instant
            { -1.0, -0.2, -0.2 },  // slowing, going backward: instant
            { 0.5, -1.0, -0.1 },   // reversing: zero first, then one step the other way
            { -0.5, 1.0,  0.1 },
            { 0.3,  0.3,  0.3 },   // no change
            { 0.0,  0.0,  0.0 },
        };
        for (double[] row : table) {
            double got = (Double) ramp.invoke(drive, row[0], row[1], 0.1);
            check("rampToward(current " + row[0] + ", wanted " + row[1] + ")", Math.abs(got - row[2]) < 1e-9,
                  "-> " + got + " (expected " + row[2] + ")");
        }

        // --- a real ramp, over real time: full stick forward, 20 ms loops ---
        System.out.println("--- ramp: stick to full forward ---");
        fresh();
        loop20ms(1, 0, 0, 1);
        double firstStep = lastPower[0];
        check("first step is small, not a jump to 1.0", firstStep > 0.0 && firstStep <= 0.25 + 1e-9, "first step = " + firstStep);
        double previous = firstStep; boolean rising = true; double biggestJump = firstStep; int loops = 1;
        while (lastPower[0] < 1.0 && loops < 60) {
            loop20ms(1, 0, 0, 1); loops++;
            if (lastPower[0] < previous - 1e-12) rising = false;
            biggestJump = Math.max(biggestJump, lastPower[0] - previous);
            previous = lastPower[0];
        }
        check("power only rises", rising, "");
        check("biggest single jump stays within one capped step (0.25)", biggestJump <= 0.25 + 1e-9, "biggest jump = " + biggestJump);
        check("all four wheels got the same ramp", Math.abs(lastPower[0]-lastPower[1])<1e-9 && Math.abs(lastPower[2]-lastPower[3])<1e-9, "");
        check("reaches full power in a fraction of a second (not 1 loop)", loops > 3 && loops <= 25, "took " + loops + " loops of ~20 ms");

        // --- instant stop and instant slow-down ---
        System.out.println("--- stopping and slowing ---");
        loop20ms(0.5, 0, 0, 1);
        check("slowing from 1.0 to 0.5 is instant", Math.abs(lastPower[0] - 0.5) < 1e-9, "power = " + lastPower[0]);
        loop20ms(0, 0, 0, 1);
        check("letting go of the stick stops at once", lastPower[0] == 0.0 && lastPower[1] == 0.0 && lastPower[2] == 0.0 && lastPower[3] == 0.0, "");

        // --- reversal goes through zero ---
        System.out.println("--- reversing ---");
        fresh();
        loop20ms(1, 0, 0, 25);                       // get to full forward
        check("was at full forward", Math.abs(lastPower[0] - 1.0) < 1e-9, "power = " + lastPower[0]);
        loop20ms(-1, 0, 0, 1);
        check("reversal: first step is NOT a jump to -1.0, and not still forward",
              lastPower[0] <= 0.0 && lastPower[0] >= -0.25 - 1e-9, "power = " + lastPower[0]);

        // --- the turret's feedforward number ---
        System.out.println("--- getTurnPower() follows the REAL spin, not the stick ---");
        fresh();
        loop20ms(0, 0, 0.6, 1);
        double turnEarly = drive.getTurnPower();
        check("right after the stick moves, the real turn is still building (less than 0.6)", turnEarly > 0.0 && turnEarly < 0.6, "turn = " + turnEarly);
        loop20ms(0, 0, 0.6, 20);
        check("once built up, it matches the stick (0.6)", Math.abs(drive.getTurnPower() - 0.6) < 1e-9, "turn = " + drive.getTurnPower());
        fresh();
        loop20ms(0.5, 0.5, 0.0, 25);
        check("driving without turning reports 0 turn", Math.abs(drive.getTurnPower()) < 1e-9, "turn = " + drive.getTurnPower());
        drive.stop();
        check("stop() puts every wheel and the turn at 0", drive.getTurnPower() == 0.0 && lastPower[0] == 0.0, "");

        System.out.println(failures == 0 ? "\nALL PASSED" : "\n" + failures + " FAILED");
        System.exit(failures == 0 ? 0 : 1);
    }
}
