package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    /**
     * Command to spin up the shooter
     * @param shooterSubsystem Instance of {@link ShooterSubsystem}
     */
    public SpinUpCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setSpeed(6000);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setSpeed(0);
    }
}
