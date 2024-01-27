package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.constants.VisionConstants;
import frc.robot.utils.VisionUtils;

public class AlignCommand extends Command {
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

    @Override
    public void execute() {
        Pose3d botPoseTargetSpace = VisionUtils.getBotPoseTargetSpace();

        if (botPoseTargetSpace.getX() != 0 || botPoseTargetSpace.getY() != 0) {
            latestX = botPoseTargetSpace.getX();

            // Coordinate systems are different
            // Pose2D is X/Y, and Y is equivalent to Z in botPoseTargetSpace
            latestZ = -botPoseTargetSpace.getZ();
            latestRotation = botPoseTargetSpace.getRotation().toRotation2d().getRadians();

            System.out.println("--------------------------");
            System.out.println("Robot thinks its at: " + botPoseTargetSpace);
            System.out.println("Robot XPID: " + pidXController.calculate(latestX));
            System.out.println("Robot ZPID: " + pidZController.calculate(latestZ));
            System.out.println("Robot RPID: " + pidRotationController.calculate(latestRotation));

            swerveSubsystem.drive(
                    pidXController.calculate(latestX) / 4.0,
                    pidZController.calculate(latestZ) / 4.0,
                    pidRotationController.calculate(latestRotation) / 4.0,
                    false, false
            );
        } else {
            System.out.println("Vision could not find a target!");
        }
    }

    @Override
    public void end(boolean x) {
        swerveSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }
}
