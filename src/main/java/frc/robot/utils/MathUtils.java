package frc.robot.utils;

public class MathUtils {

    public static double rpmToRadians(double rpm) {
        return rpm * (2 * Math.PI/60);
    }
}
