package android.util;
public class Log {
    public static int v(String t, String m) { return 0; } public static int d(String t, String m) { return 0; }
    public static int i(String t, String m) { System.out.println("   [log i] " + m); return 0; }
    public static int w(String t, String m) { System.out.println("   [log w] " + m); return 0; }
    public static int e(String t, String m) { return 0; }
    public static int v(String t, String m, Throwable x) { return 0; } public static int d(String t, String m, Throwable x) { return 0; }
    public static int i(String t, String m, Throwable x) { return 0; } public static int w(String t, String m, Throwable x) { return 0; }
    public static int e(String t, String m, Throwable x) { return 0; }
    public static int println(int priority, String tag, String msg) { System.out.println("   [log " + priority + "] " + msg); return 0; }
    public static String getStackTraceString(Throwable t) { return String.valueOf(t); }
    public static boolean isLoggable(String t, int l) { return true; }
}
