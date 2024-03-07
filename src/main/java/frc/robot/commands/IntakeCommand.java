package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import java.rmi.MarshalException;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    private final IndexerSubsystem indexerSubsystem;

    private final Targets target;

    private final boolean source;

    //private LEDSubsystem ledSubsystem;

    /**
     * Command to run the intake
     * @param intakeSubsystem The instance of {@link IntakeSubsystem}
     * @param indexerSubsystem The instance of {@link IndexerSubsystem} (needed for limebreak detection to stop intake motor)
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, Targets target, boolean source) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.target = target;
        this.source = source;

        addRequirements(intakeSubsystem, indexerSubsystem);
    }

    @Override
    public void execute() {
//        if (!indexerSubsystem.isCenter()) {
//            intakeSubsystem.setSpeed(.9);
//            indexerSubsystem.rotateAllWheelsPercent(.9);
//            ledSubsystem.setLED(-0.71);
//        } else {
//            intakeSubsystem.setSpeed(0);
//            indexerSubsystem.rotateAllWheelsPercent(0);
//        }
        if (source) {
            indexerSubsystem.moveIndexerToPos(Math.toRadians(140));
        } else {
//            indexerSubsystem.moveIndexerToPos(Math.toRadians(7.0));
//            indexerSubsystem.rotateMotorVolts(IndexerSubsystem.IndexerMotors.INDEXER_ROTATE, 0.0);
        }

        switch (target) {
            case AMP -> {
//                if (!indexerSubsystem.isTop()) {
                    indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.TOP_WHEEL, 0.22);
                    indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.BOTTOM_WHEELS, -0.22);
                    intakeSubsystem.setTopSpeed(-0.4);
                    intakeSubsystem.setBottomSpeed(-0.7);
//                } else if (indexerSubsystem.isTop()) {
                    indexerSubsystem.rotateAllWheelsPercent(0.0);
                    intakeSubsystem.setSpeed(0.0);
//                }
            }
            case SPEAKER -> {
                if (!indexerSubsystem.isCenter()) {
                    intakeSubsystem.setTopSpeed(0.4);
                    intakeSubsystem.setBottomSpeed(0.4);
                    indexerSubsystem.rotateAllWheelsPercent(0.3);
                } else if (indexerSubsystem.isCenter()) {
                   // intakeSubsystem.setSpeed(0);
                    try {
                        Thread.sleep(105);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if (indexerSubsystem.isCenter()) {
                        indexerSubsystem.rotateAllWheelsPercent(0);
                        intakeSubsystem.setSpeed(0.0);
                    }

                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0);
        indexerSubsystem.rotateAllWheelsPercent(0);
    }

    public enum Targets {
        AMP,
        SPEAKER
    }
}
