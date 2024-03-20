package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class JiggleCommand extends Command {
    private IndexerSubsystem indexerSubsystem;

    private IntakeSubsystem intakeSubsystem;


    private int jiggleCount = 0;

    private double currentTime;

    private int state;

    public JiggleCommand(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(indexerSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        if (indexerSubsystem.isCenter()) {
            intakeSubsystem.setTopSpeed(0.3);
            intakeSubsystem.setBottomSpeed(-0.3);
            if (jiggleCount <= 11) {
                double speed = .15;
                if (Timer.getFPGATimestamp() - currentTime > 0.28) {
                    if (state == 0) {
                        indexerSubsystem.rotateAllWheelsPercent(speed);
                        state = 1;
                    } else if (state == 1) {
                        indexerSubsystem.rotateAllWheelsPercent(-speed);
                        state = 0;
                    }
                    jiggleCount += 1;
                    currentTime = Timer.getFPGATimestamp();
                }
            } else {
                indexerSubsystem.rotateAllWheelsPercent(0.0);
                intakeSubsystem.setSpeed(0.0);
            }
        }  else {
            indexerSubsystem.rotateAllWheelsPercent(0.0);
            intakeSubsystem.setSpeed(0.0);
            jiggleCount = 0;
        }
    }

    @Override
    public void initialize() {
        currentTime = Timer.getFPGATimestamp();
        state = 0;
        jiggleCount = 0;
    }
}
