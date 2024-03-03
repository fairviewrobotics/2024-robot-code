package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    private final boolean advanced;

    private Pose2d robotPose;

    /**
     * Command to spin up the shooter, simply just spins the motors to shooterRPM (defined in {@link ShooterConstants})
     * @param shooterSubsystem Instance of {@link ShooterSubsystem}
     * @param indexerSubsystem Instance of {@link IndexerSubsystem}
     */
    public SpinUpCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, boolean advanced) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.advanced = advanced;

        addRequirements(shooterSubsystem, indexerSubsystem);
    }


    /**
     * Spins up the shooter if we are close to the speaker
     *
     * @param shooterSubsystem Instance of the {@link ShooterSubsystem}
     * @param indexerSubsystem
     * @param robotPose        Pose2d of the robots current position
     */
    public SpinUpCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, Pose2d robotPose) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.robotPose = robotPose;
        this.advanced = true;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        if (advanced) {
            if (robotPose.getX() < 9) {
                if (indexerSubsystem.isCenter()) {
                    shooterSubsystem.setSpeed(ShooterConstants.shooterRPM);
                } else if (!indexerSubsystem.isCenter()) {
                    try {
                        Thread.sleep(3005);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if (!indexerSubsystem.isCenter()) {
                        shooterSubsystem.setVoltage(0,0);
                    }

                }
            } else {
                shooterSubsystem.setVoltage(0,0);
            }
        } else {
             shooterSubsystem.setSpeed(ShooterConstants.shooterRPM);
         }
    }

    @Override
    public void initialize() {
        ShooterConstants.isActive = true;
    }

    @Override
    public void end(boolean interrupted) {
        ShooterConstants.isActive = false;
        shooterSubsystem.setVoltage(0);
    }
}
