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
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.rotateAllWheelsPercent(0.0);
        if (indexerSubsystem.getIndexerAngle() > Math.toRadians(75)) {
            indexerSubsystem.moveIndexerToPos(Math.toRadians(75.0));
        } else {
            indexerSubsystem.rotateMotorVolts(IndexerSubsystem.IndexerMotors.INDEXER_ROTATE, 0.0);
       }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateMotorVolts(IndexerSubsystem.IndexerMotors.INDEXER_ROTATE, 0.0);
    }
}
