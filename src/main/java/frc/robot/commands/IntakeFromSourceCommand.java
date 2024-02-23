package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeFromSourceCommand extends Command {

    private final IndexerSubsystem indexerSubsystem;

    /**
     * This command intakes a note from the source/human player instead of the floor
     * @param indexerSubsystem Instance of {@link IndexerSubsystem}
     */
    public IntakeFromSourceCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.moveIndexerToPos(160);
        if (!indexerSubsystem.isCenter()) {
            indexerSubsystem.rotateAllWheelsPercent(0.4);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateAllWheelsPercent(0);
    }
}
