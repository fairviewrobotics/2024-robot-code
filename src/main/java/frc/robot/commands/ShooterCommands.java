package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommands extends Command {

    public final ShooterSubsystem shooterSubsystem;
    public final double topPercent;
    public final double bottomPercent;

    public ShooterCommands(ShooterSubsystem shooterSubsystem, double topPercent, double bottomPercent) {
        this.shooterSubsystem = shooterSubsystem;
        this.topPercent = topPercent;
        this.bottomPercent = bottomPercent;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.percentShoot(topPercent, bottomPercent);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.voltageShoot(0.0, 0.0);
    }

}
