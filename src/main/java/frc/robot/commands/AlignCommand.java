package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.constants.VisionConstants;
import frc.robot.utils.VisionUtils;

public class AlignCommand {
    private final ProfiledPIDController pidXController = new ProfiledPIDController(
            VisionConstants.alignXP, VisionConstants.alignXI, VisionConstants.alignXD,
            VisionConstants.alignXConstraints
    );

    private final ProfiledPIDController pidZController = new ProfiledPIDController(
            VisionConstants.alignZP, VisionConstants.alignZI, VisionConstants.alignZD,
            VisionConstants.alignZConstraints
    );
    private final ProfiledPIDController pidRotationController = new ProfiledPIDController(
            VisionConstants.alignRotationP, VisionConstants.alignRotationI, VisionConstants.alignRotationD,
            VisionConstants.alignRotationConstraints
    );

    private double latestX = 0.0;
    private double latestZ = 0.0;
    private double latestRotation = 0.0;
    private SwerveSubsystem swerveSubsystem;

    public AlignCommand(SwerveSubsystem swerveSubsystem, double desiredX, double desiredZ, double desiredRotation) {
        setupPIDController(pidXController, desiredX, latestX);
        setupPIDController(pidZController, desiredZ, latestZ);
        setupPIDController(pidRotationController, desiredRotation, latestRotation);

        this.swerveSubsystem = swerveSubsystem;
    }

    private void setupPIDController(ProfiledPIDController controller, double desired, double latest) {
        controller.setTolerance(0.1, 0.1);
        controller.reset(latest);
        controller.setGoal(desired);
    }

    public void execute() {
        Pose2d botPoseTargetSpace = VisionUtils.getBotPoseTargetSpace();

        if (botPoseTargetSpace.getX() != 0 || botPoseTargetSpace.getY() != 0) {
            latestX = botPoseTargetSpace.getX();

            // Coordinate systems are different
            // Pose2D is X/Y, and Y is equivalent to Z in botPoseTargetSpace
            latestZ = botPoseTargetSpace.getY();
            latestRotation = botPoseTargetSpace.getRotation().getRadians();

            System.out.println("Latest X: " + latestX + "m");
            System.out.println("Latest Z (Y): " + latestZ + "m");
            System.out.println("Latest Rotation: " + latestRotation + "rad");

//            swerveSubsystem.drive(
//                    pidXController.calculate(latestX),
//                    pidZController.calculate(latestZ),
//                    pidRotationController.calculate(latestRotation),
//                    false, false
//            );
        } else {
            System.out.println("Vision could not find a target!");
        }
    }

    public void end() {
        swerveSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }
}
