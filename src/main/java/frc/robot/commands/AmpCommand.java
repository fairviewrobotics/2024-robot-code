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

        while (!indexerSubsystem.isTop()) {
            indexerSubsystem.topWheel.set(0.22);
            indexerSubsystem.bottomWheels.set(-0.22);
        }
        if (indexerSubsystem.isTop()) {
            indexerSubsystem.topWheel.set(0.0);
            indexerSubsystem.bottomWheels.set(0.0);
           indexerSubsystem.moveIndexerToPos(140);

            //the following doesn;t work yet
            if (xboxController.getRightBumper() && indexerSubsystem.isIndexerRotated()) {
                indexerSubsystem.rotateAllWheelsPercent(-0.5);
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateAllWheelsPercent(0);
    }
}
