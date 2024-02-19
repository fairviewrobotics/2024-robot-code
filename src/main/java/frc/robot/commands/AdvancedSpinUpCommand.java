package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AdvancedSpinUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    private final Pose2d robotPose;

    /**
     * Command to spin up the shooter
     * @param shooterSubsystem Instance of {@link ShooterSubsystem}
     * @param robotPose Instance of {@link Pose2d}
     */
    public AdvancedSpinUpCommand(ShooterSubsystem shooterSubsystem, Pose2d robotPose) {
        this.shooterSubsystem = shooterSubsystem;
        this.robotPose = robotPose;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {

        if (robotPose.getX() < 9) {
            shooterSubsystem.setSpeed(ShooterConstants.shooterRPM);
        } else {
            shooterSubsystem.setVoltage(0,0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setVoltage(0,0);
    }
}
