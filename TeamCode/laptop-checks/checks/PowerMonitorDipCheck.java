import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.PowerMonitorSubsystem;

public class PowerMonitorDipCheck {
    static Object monitor;
    static Method updateDipLog;
    static int failures = 0;

    static void setField(String name, Object value) throws Exception {
        Field f = PowerMonitorSubsystem.class.getDeclaredField(name);
        f.setAccessible(true);
        f.set(monitor, value);
    }
    static Object getField(String name) throws Exception {
        Field f = PowerMonitorSubsystem.class.getDeclaredField(name);
        f.setAccessible(true);
        return f.get(monitor);
    }
    static void feed(double volts) throws Exception { updateDipLog.invoke(monitor, volts, "Control Hub"); }
    static void check(String what, Object actual, Object expected) {
        boolean ok = actual.equals(expected);
        if (!ok) failures++;
        System.out.println((ok ? "PASS " : "FAIL ") + what + " -> " + actual + (ok ? "" : "  (wanted " + expected + ")"));
    }
    static void fresh() throws Exception {
        java.lang.reflect.Constructor<?> unused = null;
        Field uf = sun.misc.Unsafe.class.getDeclaredField("theUnsafe"); uf.setAccessible(true);
        sun.misc.Unsafe unsafe = (sun.misc.Unsafe) uf.get(null);
        monitor = unsafe.allocateInstance(PowerMonitorSubsystem.class);   // skips the constructor (it needs a HardwareMap)
        setField("dipTimer", new ElapsedTime());
        setField("sinceStartTimer", new ElapsedTime());
        setField("lowestInThisDip", 99.0);
        setField("lowestEverVolts", 99.0);
        setField("lastDipText", "none");
    }

    public static void main(String[] args) throws Exception {
        updateDipLog = PowerMonitorSubsystem.class.getDeclaredMethod("updateDipLog", double.class, String.class);
        updateDipLog.setAccessible(true);

        System.out.println("--- Scenario 1: one clean dip (12 -> 9.0 -> 12) ---");
        fresh();
        feed(12.4); feed(11.2); feed(9.6); feed(9.0); feed(9.8); feed(10.4); feed(12.1);
        check("dipCount", getField("dipCount"), 1);
        check("inDip at the end", getField("inDip"), false);
        check("lowest in the dip", getField("lowestInThisDip"), 9.0);

        System.out.println("--- Scenario 2: chatter around the threshold is ONE dip, not many ---");
        fresh();
        feed(12.0); feed(9.9); feed(10.1); feed(9.9); feed(10.2); feed(9.8); feed(10.25);   // never reaches 10.3
        check("dipCount (hysteresis)", getField("dipCount"), 1);
        check("still in the dip (never climbed past 10.3)", getField("inDip"), true);

        System.out.println("--- Scenario 3: two separate dips ---");
        fresh();
        feed(12.0); feed(9.5); feed(11.5); feed(12.0); feed(9.2); feed(10.9);
        check("dipCount", getField("dipCount"), 2);

        System.out.println("--- Scenario 4: a healthy battery never dips ---");
        fresh();
        for (int i = 0; i < 50; i++) feed(11.0 + (i % 5) * 0.4);
        check("dipCount", getField("dipCount"), 0);

        System.out.println("--- Scenario 5: 'no reading' (99.0 = hub did not answer) changes nothing ---");
        fresh();
        feed(12.0); feed(9.0); feed(99.0); feed(99.0);
        check("dipCount", getField("dipCount"), 1);
        check("still in the dip (no reading is not a recovery)", getField("inDip"), true);

        System.out.println(failures == 0 ? "\nALL PASSED" : "\n" + failures + " FAILED");
        System.exit(failures == 0 ? 0 : 1);
    }
}
