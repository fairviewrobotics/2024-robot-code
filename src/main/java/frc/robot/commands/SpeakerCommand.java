package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SpeakerCommand extends Command {

    private final IndexerSubsystem indexerSubsystem;


    public SpeakerCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_1, 1);
        indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_2, 1);
        indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_3, 1);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_1, 0);
        indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_2, 0);
        indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_3, 0);
    }
}
