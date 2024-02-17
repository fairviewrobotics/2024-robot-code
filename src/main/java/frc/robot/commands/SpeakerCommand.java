package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class SpeakerCommand extends Command {

    private final IndexerSubsystem indexerSubsystem;
    private final ChassisSpeeds fieldRelativeSpeeds;
    private final Pose2d swervePose;


    /**
     * Command to shoot for the speaker once there is a note in the intake
     * @param indexerSubsystem Instance of {@link IndexerSubsystem}
     * @param fieldRelativeSpeeds Instance of {@link ChassisSpeeds}
     * @param swervePose Instance of {@link Pose2d}
     */

    public SpeakerCommand(IndexerSubsystem indexerSubsystem, ChassisSpeeds fieldRelativeSpeeds, Pose2d swervePose) {
        this.indexerSubsystem = indexerSubsystem;
        this.fieldRelativeSpeeds = fieldRelativeSpeeds;
        this.swervePose = swervePose;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {

        //Robot speed toward the speaker
        double robotXSpeed = fieldRelativeSpeeds.vxMetersPerSecond;
        //Note speed in x-direction(forward, as opposed to up)
        double noteSpeedX = robotXSpeed + ShooterConstants.shooterNoteSpeedX;

        //Initial calculated distance to shoot from the speaker
        double shootingDistanceFromSpeaker = noteSpeedX * ShooterConstants.timeToSpeakerHeightWithGravity;

        //Robot distance to speaker
        double robotDistanceFromSpeaker = Math.sqrt(
                Math.pow(
                        (
                            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)?
                                    ShooterConstants.speakerPoseRed.getX():
                                    ShooterConstants.speakerPoseBlue.getX()
                        ) - swervePose.getX(),
                        2
                )
                + Math.pow(
                        (
                                (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)?
                                        ShooterConstants.speakerPoseRed.getY():
                                        ShooterConstants.speakerPoseBlue.getY()
                        ) - swervePose.getY(),
                        2
                )
        );

        //Difference between optimal location and current location
        double difference = robotDistanceFromSpeaker - shootingDistanceFromSpeaker;

        //Accounting for indexer spinup time
        if (difference > 0) {
            shootingDistanceFromSpeaker += noteSpeedX * ShooterConstants.shootDelayTime;
        } else {
            shootingDistanceFromSpeaker -= noteSpeedX * ShooterConstants.shootDelayTime;
        }

        //Removing negatives
        difference = Math.abs(robotDistanceFromSpeaker - shootingDistanceFromSpeaker);

        System.out.println("Robot Speed: " + robotXSpeed +
                "\n Note Speed: " + noteSpeedX +
                "\n Calculated Distance To Shoot: " + shootingDistanceFromSpeaker +
                "\n Robot Distance: " + robotDistanceFromSpeaker+
                "\n Difference: " + difference
        );

        //Execute shoot
        if (difference < 0.4){
            indexerSubsystem.rotateAllWheelsPercent(1.0);
            System.out.println("Shooting!!!!!");
        }

        System.out.println("----------------------------------------");
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateAllWheelsPercent(0);
    }
}
