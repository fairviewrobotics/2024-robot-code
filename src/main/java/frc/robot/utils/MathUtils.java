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

    public static double inRange(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}
