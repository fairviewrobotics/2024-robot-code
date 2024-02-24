package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.VisionUtils;

import java.util.function.DoubleSupplier;

public class RotateAndDriveCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final XboxController controller;
    private final Targets target;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;
    private final DoubleSupplier radians;
    private final double rotation;

    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
            VisionConstants.rotateToP,
            VisionConstants.rotateToI,
            VisionConstants.rotateToD,
            VisionConstants.rotateToConstraints
    );


    /**
     * Rotates the robot to face the speaker, while still allowing the driver to control forward and backward movement
     * @param swerveSubsystem The instance of {@link SwerveSubsystem}
     * @param controller The instance of {@link XboxController}
     * @param forward The desired forward percentage of the robot
     * @param sideways The desired sideways percentage of the robot
     * @param radians The desired rotation speed of the robot
     */
    public RotateAndDriveCommand(SwerveSubsystem swerveSubsystem,
                                 XboxController controller,
                                 Targets targets,
                                 DoubleSupplier forward,
                                 DoubleSupplier sideways,
                                 DoubleSupplier radians) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        this.target = targets;
        this.forward = forward;
        this.sideways = sideways;
        this.radians = radians;

        rotationPID.setTolerance(0.05);

        double angle = getAngle(swerveSubsystem);

        this.rotation = target == Targets.NOTE ?
                rotationPID.calculate(VisionUtils.getNoteTX(), 0.0) :
                rotationPID.calculate(swerveSubsystem.getPose().getRotation().getRadians(), angle);

        addRequirements(swerveSubsystem);
    }

    private static double getAngle(SwerveSubsystem swerveSubsystem) {
        double YSpeed = swerveSubsystem.getFieldRelativeChassisSpeeds().vyMetersPerSecond;
        double XSpeed = swerveSubsystem.getFieldRelativeChassisSpeeds().vxMetersPerSecond;
        double noteXSpeed = XSpeed + ShooterConstants.shooterNoteSpeedX;

        Pose2d speakerPos = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.speakerPoseBlue : ShooterConstants.speakerPoseRed;

        double speakerYTranslation = YSpeed * ((speakerPos.getX()- swerveSubsystem.getPose().getX()) / noteXSpeed);

        return Math.atan2((speakerPos.getY() + speakerYTranslation) - swerveSubsystem.getPose().getY(), speakerPos.getX()- swerveSubsystem.getPose().getX());
    }

    @Override
    public void execute() {
        switch (target) {
            case NOTE -> {
                if (VisionUtils.getNoteTV() == 1.0) {
                    swerveSubsystem.drive(
                            forward.getAsDouble(),
                            sideways.getAsDouble(),
                            rotation,
                            true,
                            true
                    );
                    controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                } else {
                    swerveSubsystem.drive(
                            forward.getAsDouble(),
                            sideways.getAsDouble(),
                            radians.getAsDouble(),
                            true,
                            true
                    );
                    controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
                }
            }
            case SPEAKER -> {
                swerveSubsystem.drive(
                        forward.getAsDouble(),
                        sideways.getAsDouble(),
                        rotation,
                        true,
                        true
                );
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

    public enum Targets {
        NOTE,
        SPEAKER
    }
}
