package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;


    private Pose2d robotPose;

    /**
     * Command to spin up the shooter, simply just spins the motors to shooterRPM (defined in {@link ShooterConstants})
     * @param shooterSubsystem Instance of {@link ShooterSubsystem}
     */
    public SpinUpCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;


        addRequirements(shooterSubsystem);
    }


//    /**
//     * Spins up the shooter if we are close to the speaker
//     *
//     * @param shooterSubsystem Instance of the {@link ShooterSubsystem}
//     * @param indexerSubsystem
//     * @param robotPose        Pose2d of the robots current position
//     */
//    public SpinUpCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, Pose2d robotPose) {
//        this.shooterSubsystem = shooterSubsystem;
//        this.indexerSubsystem = indexerSubsystem;
//        this.robotPose = robotPose;
//        this.advanced = true;
//
//        addRequirements(shooterSubsystem);
//    }

    @Override
    public void execute() {
        shooterSubsystem.setSpeed(ShooterConstants.shooterRPM);
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
