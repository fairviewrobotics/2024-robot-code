package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class BaseCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;

    /**
     * Reset indexer to default pos
     * @param indexerSubsystem Instance of {@link IndexerSubsystem}
     */
    public BaseCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
    }

    @Override
    public void execute() {
        if (indexerSubsystem.getIndexerAngle() > Math.PI/6) {
            indexerSubsystem.moveIndexerToPos(10);
        } else {
            indexerSubsystem.rotateMotorVolts(IndexerSubsystem.IndexerMotors.INDEXER_POS, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateMotorVolts(IndexerSubsystem.IndexerMotors.INDEXER_POS, 0.0);
    }
}
