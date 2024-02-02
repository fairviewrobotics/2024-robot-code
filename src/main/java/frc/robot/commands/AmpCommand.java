package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class AmpCommand extends Command {

    private final IndexerSubsystem indexerSubsystem;

    private final XboxController xboxController;

    private boolean reversed = false;


    public AmpCommand(IndexerSubsystem indexerSubsystem, XboxController controller) {
        this.indexerSubsystem = indexerSubsystem;
        this.xboxController = controller;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        if (indexerSubsystem.getCenterLimebreak() && !reversed) {
            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_1, 0.5);
            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_2, 0.5);
            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_3, 0.5);
        } else {
            reversed = true;
        }

        if (reversed && !indexerSubsystem.getTopLimebreak()) {
            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_1, -0.5);
            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_2, -0.5);
            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_3, -0.5);
        } else if (reversed && indexerSubsystem.getTopLimebreak()) {
            indexerSubsystem.moveIndexerToPos(160);
        }

        if (xboxController.getRightBumper() && indexerSubsystem.isIndexerRotated()) {
            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_1, -0.5);
            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_2, -0.5);
            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_3, -0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_1, 0.0);
        indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_2, 0.0);
        indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_3, 0.0);
    }
}
