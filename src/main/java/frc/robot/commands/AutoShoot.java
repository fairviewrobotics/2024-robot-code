package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class AutoShoot extends Command {
    public IndexerSubsystem indexerSubsystem;

    public AutoShoot(IndexerSubsystem indexerSubsystem) {
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.rotateAllWheelsPercent(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateAllWheelsVolts(0.0);
    }
}
