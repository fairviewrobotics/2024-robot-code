package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class SpeakerCommand extends Command {

    private final IndexerSubsystem indexerSubsystem;

    /**
     * Command to shoot for the speaker once there is a note in the intake
     * @param indexerSubsystem Instance of {@link IndexerSubsystem}
     */

    public SpeakerCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.rotateAllWheelsPercent(1);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateAllWheelsPercent(0);
    }
}
