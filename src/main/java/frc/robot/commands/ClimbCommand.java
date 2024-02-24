package frc.robot.commands;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class ClimbCommand extends Command {

    private ClimberSubsystem climberSubsystem;

    private IndexerSubsystem indexerSubsystem;

    private RelativeEncoder leftEncoder;

    private RelativeEncoder rightEncoder;

    private Mode mode;

    private boolean isIndexer;

    public ClimbCommand(ClimberSubsystem climberSubsystem, Mode mode) {
        this.climberSubsystem = climberSubsystem;
        this.mode = mode;

        this.leftEncoder = climberSubsystem.getLeftEncoder();
        this.rightEncoder = climberSubsystem.getRightEncoder();
        this.isIndexer = false;

        addRequirements(climberSubsystem);
    }

    public ClimbCommand(IndexerSubsystem indexerSubsystem, Mode mode) {
        this.indexerSubsystem = indexerSubsystem;

        this.mode = mode;
        this.isIndexer = false;

        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        switch (mode) {
            case EXTEND -> {
                if (isIndexer) {
                    indexerSubsystem.moveIndexerToPos(90);
                } else {
                    double leftEncoderPos = leftEncoder.getPosition();
//                rightEncoderPos = rightEncoder.getPosition();

                    if (leftEncoderPos > 3.0) {
                        climberSubsystem.hold();
                    } else {
                        climberSubsystem.extend();
                    }
                }
            }

            case RETRACT -> {
                if (!isIndexer) {
                    double leftEncoderPos = leftEncoder.getPosition();
//              double rightEncoderPos = rightEncoder.getPosition();

                    climberSubsystem.retract();

                    if (leftEncoderPos < 1.0) {
                        climberSubsystem.hold();
                    } else {
                        climberSubsystem.retract();
                    }
                }
            }
        }
    }

    public enum Mode {
        EXTEND,
        RETRACT
    }
}
