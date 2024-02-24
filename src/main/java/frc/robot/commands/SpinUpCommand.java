package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    private final boolean advanced;

    private final Pose2d robotPose;

    /**
     * Command to spin up the shooter
     * @param shooterSubsystem Instance of {@link ShooterSubsystem}
     */
    public SpinUpCommand(ShooterSubsystem shooterSubsystem, Pose2d robotPose, boolean advanced) {
        this.shooterSubsystem = shooterSubsystem;
        this.advanced = advanced;
        this.robotPose = robotPose;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        if (advanced) {
            if (robotPose.getX() < 9) {
                shooterSubsystem.setSpeed(ShooterConstants.shooterRPM);
            } else {
                shooterSubsystem.setVoltage(0,0);
            }
        } else {
             shooterSubsystem.setSpeed(ShooterConstants.shooterRPM);
         }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setVoltage(0);
    }
}
