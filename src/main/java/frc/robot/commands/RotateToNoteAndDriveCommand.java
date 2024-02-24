package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.VisionUtils;
import java.util.function.DoubleSupplier;

public class RotateToNoteAndDriveCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;
    private final DoubleSupplier radians;
    private final double visionRotation;

    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
            VisionConstants.rotateToP,
            VisionConstants.rotateToI,
            VisionConstants.rotateToD,
            VisionConstants.rotateToConstraints
    );

    private final XboxController controller;

    /**
     * Rotates the robot to face the speaker, while still allowing the driver to control forward and backward movement
     * @param swerveSubsystem The instance of {@link SwerveSubsystem}
     * @param controller The instance of {@link XboxController}
     * @param forward The desired forward percentage of the robot
     * @param sideways The desired sideways percentage of the robot
     * @param radians The desired rotation speed of the robot
     */
    public RotateToNoteAndDriveCommand(SwerveSubsystem swerveSubsystem, XboxController controller, DoubleSupplier forward, DoubleSupplier sideways, DoubleSupplier radians) {
        this.swerveSubsystem = swerveSubsystem;
        this.forward = forward;
        this.sideways = sideways;
        this.radians = radians;
        this.controller = controller;

        rotationPID.setTolerance(0.05);

        this.visionRotation = rotationPID.calculate(VisionUtils.getNoteTX(), 0.0);

        addRequirements(swerveSubsystem);
    }


    @Override
    public void execute() {
        if (VisionUtils.getNoteTV() == 1.0) {
            swerveSubsystem.drive(
                    forward.getAsDouble(),
                    sideways.getAsDouble(),
                    visionRotation,
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

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
}
