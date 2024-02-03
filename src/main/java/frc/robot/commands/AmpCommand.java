package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class AmpCommand extends Command {

    private final IndexerSubsystem indexerSubsystem;

    private final XboxController xboxController;

    private boolean reversed = false;


    /**
     * Moves the indexer to the amp
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
        if (indexerSubsystem.isCenter() && !reversed) {
            indexerSubsystem.rotateAllWheelsPercent(0.5);
        } else {
            reversed = true;
        }

        if (reversed && !indexerSubsystem.isTop()) {
            indexerSubsystem.rotateAllWheelsPercent(-0.5);
        } else if (reversed && indexerSubsystem.isTop()) {
            indexerSubsystem.moveIndexerToPos(160);
        }

        if (xboxController.getRightBumper() && indexerSubsystem.isIndexerRotated()) {
            indexerSubsystem.rotateAllWheelsPercent(-0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateAllWheelsPercent(0);
    }
}
