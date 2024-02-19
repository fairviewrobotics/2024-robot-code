package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.VisionUtils;

public class AutoRotateToSpeakerAndShoot extends Command {
    private final double odometryRotation;

    private SwerveSubsystem swerveSubsystem;

    private IndexerSubsystem indexerSubsystem;

    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
            VisionConstants.rotateToP,
            VisionConstants.rotateToI,
            VisionConstants.rotateToD,
            VisionConstants.rotateToConstraints
    );

    public AutoRotateToSpeakerAndShoot(SwerveSubsystem swerveSubsystem, IndexerSubsystem indexerSubsystem) {

        this.swerveSubsystem = swerveSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        rotationPID.setTolerance(0.05);

        Pose2d speakerPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.speakerPoseBlue : ShooterConstants.speakerPoseRed;
        //Current robot pose
        Pose2d robotPose = swerveSubsystem.getPose();

        //'Moving' speaker position based on robot speed


        //Calculated angle to rotate to
        double angle = Math.atan2(speakerPose.getY() - robotPose.getY(), speakerPose.getX()-robotPose.getX());

        //Rotation PID Calculations
        this.odometryRotation = rotationPID.calculate(robotPose.getRotation().getRadians(), angle);


        addRequirements(swerveSubsystem);

    }

    @Override
    public void execute() {
        if (!rotationPID.atGoal()) {
            swerveSubsystem.drive(
                    0.0,
                    0.0,
                    odometryRotation,
                    true,
                    true
            );
        } else {
            indexerSubsystem.rotateAllWheelsPercent(1.0);
        }

    }
}
