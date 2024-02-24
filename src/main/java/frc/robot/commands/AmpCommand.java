package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class AmpCommand extends Command {

    private final IndexerSubsystem indexerSubsystem;

    private final XboxController xboxController;

    /**
     * Moves the indexer to the amp and scores
     * @param indexerSubsystem The instance of {@link IndexerSubsystem}
     * @param controller An instance of {@link XboxController}
     */
    public AmpCommand(IndexerSubsystem indexerSubsystem, XboxController controller) {
        this.indexerSubsystem = indexerSubsystem;
        this.xboxController = controller;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        if (indexerSubsystem.isTop()) {
            indexerSubsystem.rotateAllWheelsPercent(0.0);
            indexerSubsystem.moveIndexerToPos(120);

            if (xboxController.getRightBumper()) {
                if (indexerSubsystem.isCenter()) {
                    indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.TOP_WHEEL, 0.25);
                    indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.BOTTOM_WHEELS, -0.25);
                }
            } else {
                indexerSubsystem.rotateAllWheelsPercent(0.0);
                indexerSubsystem.moveIndexerToPos(0.0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateAllWheelsPercent(0);
    }
}
