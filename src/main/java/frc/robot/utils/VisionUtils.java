package frc.robot.utils;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DriverStation;

public class VisionUtils {
    /**
     * Gets the requested entry from as well as optionally adding the alliance. Used internally
     * @param pose The pose you want as defined by https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data
     * @param useAlliance Whether to add on the alliance
     * @return The pose from the limelight
     */
    private static Pose3d getPose(String pose, boolean useAlliance) {
        String suffix = (useAlliance && DriverStation.getAlliance().isPresent()) ?
                ((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? "_wpiblue" : "_wpired") : "";

        double[] returnedPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry(pose + suffix).getDoubleArray(new double[0]);
        if (returnedPose.length == 0) return new Pose3d();

        return new Pose3d(
                new Translation3d(returnedPose[0], returnedPose[1], returnedPose[2]),
                new Rotation3d(0.0, 0.0, Math.toRadians(returnedPose[5]))
        );
    }

    /**
     * Gets the bots position relative to the field. Used by odometry
     * @return The bots position
     * */
    public static Pose3d getBotPoseFieldSpace() {
        return getPose("botpose", true);
    }

    public static double getDistanceFromTag() {
        double[] returnedPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[0]);

        return -returnedPose[2];
    }

    /**
     * Gets the bots position relative to the target. Used by AlignCommand
     * @return The bots position
     * */
    public static Pose3d getBotPoseTargetSpace() {
        return getPose("botpose_targetspace", false);
    }

    /**
     * Gets the latency from the pipeline
     * @return The latency
     */
    public static double getLatencyPipeline() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0.0);
    }

    /**
     * Gets the latency from capturing
     * @return The latency
     */
    public static double getLatencyCapture() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0.0);
    }
}
