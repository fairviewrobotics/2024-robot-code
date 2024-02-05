package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class AmpCommand2 extends Command {
    private final IndexerSubsystem indexerSubsystem;

    private final XboxController xboxController;

    /**
     * Second Amp command to go straight to amp position from intake, we cannot change to speaker mode if we use this command
     * @param indexerSubsystem Instance of {@link IndexerSubsystem}
     */
    public AmpCommand2(IndexerSubsystem indexerSubsystem, XboxController xboxController) {
        this.indexerSubsystem = indexerSubsystem;
        this.xboxController = xboxController;

        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
       if (!indexerSubsystem.isTop()) {
           indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_1, 0.4);
           indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.WHEEL_2, -0.4);
       } else {
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
