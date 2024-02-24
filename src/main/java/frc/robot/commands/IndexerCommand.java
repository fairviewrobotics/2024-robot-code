package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;

    private final Mode mode;

    private XboxController controller;

    private ChassisSpeeds fieldRelativeSpeeds;

    private Pose2d swervePose;


    public IndexerCommand(IndexerSubsystem indexerSubsystem, XboxController controller) {
        this.indexerSubsystem = indexerSubsystem;
        this.controller = controller;
        this.mode = Mode.AMP;

        addRequirements(indexerSubsystem);
    }

    public IndexerCommand(IndexerSubsystem indexerSubsystem, ChassisSpeeds fieldRelativeSpeeds, Pose2d swervePose) {
        this.indexerSubsystem = indexerSubsystem;
        this.fieldRelativeSpeeds = fieldRelativeSpeeds;
        this.swervePose = swervePose;
        this.mode = Mode.SPEAKER;

        addRequirements(indexerSubsystem);
    }



    @Override
    public void execute() {
        switch (mode) {
            case AMP -> {
                if (indexerSubsystem.isTop()) {
                    indexerSubsystem.rotateAllWheelsPercent(0.0);
                    indexerSubsystem.moveIndexerToPos(120);

                    if (controller.getRightBumper()) {
                        if (indexerSubsystem.isCenter()) {
                            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.TOP_WHEEL, 0.25);
                            indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.BOTTOM_WHEELS, -0.25);
                        }
                    } else {
                        indexerSubsystem.rotateAllWheelsPercent(0.0);
                        indexerSubsystem.moveIndexerToPos(0.0);
                    }
                }
            }
            case SPEAKER -> {
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

                //Accounting for indexer spin time
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
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateAllWheelsPercent(0);
    }

    private enum Mode {
        AMP,
        SPEAKER
    }
}
