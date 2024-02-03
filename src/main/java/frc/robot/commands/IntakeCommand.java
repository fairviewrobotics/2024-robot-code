package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    private final IndexerSubsystem indexerSubsystem;

    /**
     * Command to run the intake
     * @param intakeSubsystem The instance of {@link IntakeSubsystem}
     * @param indexerSubsystem The instance of {@link IndexerSubsystem} (needed for limebreak detection to stop intake motor)
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(intakeSubsystem, indexerSubsystem);
    }

    @Override
    public void execute() {
        if (!indexerSubsystem.isCenter()) {
            intakeSubsystem.setSpeed(.9);
            indexerSubsystem.rotateAllWheelsPercent(.9);
        } else {
            intakeSubsystem.setSpeed(0);
            indexerSubsystem.rotateAllWheelsPercent(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0);
        indexerSubsystem.rotateAllWheelsPercent(0);
    }
}
