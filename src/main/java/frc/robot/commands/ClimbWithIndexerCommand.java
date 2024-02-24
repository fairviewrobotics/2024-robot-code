package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class ClimbWithIndexerCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;

    public ClimbWithIndexerCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.moveIndexerToPos(90);
    }
}
