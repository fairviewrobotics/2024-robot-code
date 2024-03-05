package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AdvancedSpinUp extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    private Pose2d robotPose;


    /**
     * Spins up the shooter if we are close to the speaker
     *
     * @param shooterSubsystem Instance of the {@link ShooterSubsystem}
     * @param indexerSubsystem
     */
    public AdvancedSpinUp(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.robotPose = robotPose;

        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    @Override
    public void execute() {
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
