package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.NetworkTableUtils;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class OnTheFlyShootCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;

    private final NetworkTableUtils debug = new NetworkTableUtils("debug");


    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
            ShooterConstants.rotateP,
            ShooterConstants.rotateI,
            ShooterConstants.rotateD,
            ShooterConstants.rotateConstraints
    );

    /**
     * Rotates the robot to face the speaker, while still allowing the driver to control forward and backward movement
     * @param swerveSubsystem The instance of {@link SwerveSubsystem}
     * @param forward The desired forward percentage of the robot
     * @param sideways The desired sideways percentage of the robot
     */
    public OnTheFlyShootCommand(SwerveSubsystem swerveSubsystem, IndexerSubsystem indexerSubsystem, DoubleSupplier forward, DoubleSupplier sideways) {
        this.swerveSubsystem = swerveSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.forward = forward;
        this.sideways = sideways;

        //Rotation tolerance in radians
        rotationPID.setTolerance(0.05);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);


        addRequirements(swerveSubsystem, indexerSubsystem);

    }

    @Override
    public void execute() {
        //ROTATION CALCULATIONS
        double odometryRotation;

        //Robot speed toward/away from the speaker (y-direction)
        double robotYSpeed = swerveSubsystem.getFieldRelativeChassisSpeeds().vyMetersPerSecond;
        //Robot speed toward/away from the speaker (x-direction)
        double robotXSpeed = swerveSubsystem.getFieldRelativeChassisSpeeds().vxMetersPerSecond;
        //Note speed in x-direction(forward, as opposed to up)
        double noteSpeedX = (ShooterConstants.shooterNoteSpeedX + Math.abs(robotXSpeed)) * 1.1;
        Pose2d speakerPose;

        //Getting speaker pose relative to alliance color
        if (DriverStation.getAlliance().isPresent()) {
            speakerPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? ShooterConstants.speakerPoseBlue
                    : ShooterConstants.speakerPoseRed;
        } else {
            speakerPose = ShooterConstants.speakerPoseBlue;
        }

        //Current robot pose
        Pose2d robotPose = swerveSubsystem.getPose();

        //'Moving' speaker position based on robot speed
        double speakerYTranslation = robotYSpeed * ((speakerPose.getX()-robotPose.getX()) / noteSpeedX);
        System.out.println("Speaker Y trans: " + speakerYTranslation);

        //Calculated angle to rotate to
        double angle = Math.atan2((speakerPose.getY() + speakerYTranslation) - robotPose.getY(), speakerPose.getX()-robotPose.getX());

        //Rotation PID Calculations
        rotationPID.setGoal(angle + Math.PI - Math.toRadians(3.0));
        odometryRotation = rotationPID.calculate(robotPose.getRotation().getRadians());





        //SHOOTING TIME CALCULATIONS

        //Initial calculated distance to shoot from the speaker
        double shootingDistanceFromSpeaker = (noteSpeedX * ShooterConstants.timeToSpeakerHeightWithGravity) + 0.1;

        System.out.println("Inital Distance Calc: " + shootingDistanceFromSpeaker);
        System.out.println("Other Distance Calc: " + ShooterConstants.shooterNoteSpeedX * ShooterConstants.timeToSpeakerHeightWithGravity);

        //Robot distance to speaker
        double robotDistanceFromSpeaker = Math.sqrt(
                Math.pow(
                        (
                                (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)?
                                        ShooterConstants.speakerPoseRed.getX():
                                        ShooterConstants.speakerPoseBlue.getX()
                        ) - swerveSubsystem.getPose().getX(),
                        2
                )
                        + Math.pow(
                        (
                                (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)?
                                        ShooterConstants.speakerPoseRed.getY():
                                        ShooterConstants.speakerPoseBlue.getY()
                        ) - swerveSubsystem.getPose().getY(),
                        2
                )
        );

        System.out.println("Robot distance from speaker " + robotDistanceFromSpeaker);

        //Difference between optimal location and current location
        double difference = robotDistanceFromSpeaker - shootingDistanceFromSpeaker;

        //Accounting for indexer spin time
        if (difference > 0) {
            shootingDistanceFromSpeaker -= robotXSpeed * ShooterConstants.shootDelayTime;
        } else {
            shootingDistanceFromSpeaker += robotXSpeed * ShooterConstants.shootDelayTime;
        }

        //Removing negatives
        difference = Math.abs(robotDistanceFromSpeaker - shootingDistanceFromSpeaker);
        debug.setDoubleArray("SpeakerPos", new double[] { ShooterConstants.speakerPoseRed.getX(), ShooterConstants.speakerPoseRed.getY() + speakerYTranslation });
        debug.setDoubleArray("ShootDist", new double[] {ShooterConstants.speakerPoseRed.getX() - shootingDistanceFromSpeaker, ShooterConstants.speakerPoseRed.getY()});

        System.out.println("Robot Speed: " + robotXSpeed +
                "\n Note Speed: " + noteSpeedX +
                "\n Calculated Distance To Shoot: " + shootingDistanceFromSpeaker +
                "\n Robot Distance: " + robotDistanceFromSpeaker+
                "\n Difference: " + difference+
                "\n Error: " + rotationPID.getPositionError()
        );
        //Actual Execute
        swerveSubsystem.drive(
                forward.getAsDouble(),
                sideways.getAsDouble(),
                odometryRotation,
                true,
                true
        );

        if (difference < 0.5 && rotationPID.atSetpoint()){
            indexerSubsystem.rotateAllWheelsPercent(0.6);
//            System.out.println("Shooting!!!!!");
        } else {
//            indexerSubsystem.rotateAllWheelsPercent(0.0);
        }

    }

    @Override
    public void initialize() {
        rotationPID.reset(swerveSubsystem.getPose().getRotation().getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.rotateAllWheelsPercent(0.0);
    }
}
