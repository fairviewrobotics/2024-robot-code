package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class RotateToSpeakerAndDriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;
    private final DoubleSupplier radians;
    private final double odometryRotation;

    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
            VisionConstants.rotateToP,
            VisionConstants.rotateToI,
            VisionConstants.rotateToD,
            VisionConstants.rotateToConstraints
    );

    /**
     * Rotates the robot to face the speaker, while still allowing the driver to control forward and backward movement
     * @param swerveSubsystem The instance of {@link SwerveSubsystem}
     * @param forward The desired forward percentage of the robot
     * @param sideways The desired sideways percentage of the robot
     * @param radians The desired rotation speed of the robot
     */
    public RotateToSpeakerAndDriveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier forward, DoubleSupplier sideways, DoubleSupplier radians) {
        this.swerveSubsystem = swerveSubsystem;
        this.forward = forward;
        this.sideways = sideways;
        this.radians = radians;

        //Rotation tolerance in radians
        rotationPID.setTolerance(0.05);

        //Robot speed toward/away from the speaker (y-direction)
        double robotYSpeed = swerveSubsystem.getFieldRelativeChassisSpeeds().vyMetersPerSecond;
        //Robot speed toward/away from the speaker (x-direction)
        double robotXSpeed = swerveSubsystem.getFieldRelativeChassisSpeeds().vxMetersPerSecond;
        //Note speed in x-direction(forward, as opposed to up)
        double noteSpeedX = robotXSpeed + ShooterConstants.shooterNoteSpeedX;

        //Getting speaker pose relative to alliance color
        Pose2d speakerPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.speakerPoseBlue : ShooterConstants.speakerPoseRed;
        //Current robot pose
        Pose2d robotPose = swerveSubsystem.getPose();

        //'Moving' speaker position based on robot speed
        double speakerYTranslation = robotYSpeed * ((speakerPose.getX()-robotPose.getX()) / noteSpeedX);

        //Calculated angle to rotate to
        double angle = Math.atan2((speakerPose.getY() + speakerYTranslation) - robotPose.getY(), speakerPose.getX()-robotPose.getX());

        //Rotation PID Calculations
        this.odometryRotation = rotationPID.calculate(robotPose.getRotation().getRadians(), angle);

        addRequirements(swerveSubsystem);

    }

    @Override
    public void execute() {

        swerveSubsystem.drive(
                forward.getAsDouble(),
                sideways.getAsDouble(),
                odometryRotation,
                true,
                true
        );

    }
}
