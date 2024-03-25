package frc.robot.utils;

public class MathUtils {

    /**
     * Converts RPM to Radians
     * @param rpm RPM value
     * @return RPM value in Rad/sec
     */
    public static double rpmToRadians(double rpm) {
        return rpm * (2 * Math.PI/60);
    }

    /**
     * Clamp a value to a minimum and a maximum
     * @param value The value to clamp
     * @param min Minimum possible value
     * @param max Maximum possible value
     * @return The value after being clamping
     */
    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}
