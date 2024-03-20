package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.NetworkTableUtils;

import java.util.function.DoubleSupplier;

public class RotateToCorner extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;

    private final Pose2d bluePose;
    private final Pose2d redPose;

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
    public RotateToCorner(SwerveSubsystem swerveSubsystem, DoubleSupplier forward, DoubleSupplier sideways, Pose2d bluePose, Pose2d redPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.forward = forward;
        this.sideways = sideways;
        this.bluePose = bluePose;
        this.redPose = redPose;

        //Rotation tolerance in radians
        rotationPID.setTolerance(0.05);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);


        addRequirements(swerveSubsystem);

    }

    @Override
    public void execute() {
        //ROTATION CALCULATIONS
        double odometryRotation;

        //Robot speed toward/away from the speaker (y-direction)

        Pose2d goalPose;

        //Getting speaker pose relative to alliance color
        if (DriverStation.getAlliance().isPresent()) {
            goalPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? bluePose
                    : redPose;
        } else {
            goalPose = bluePose;
        }

        //Current robot pose
        Pose2d robotPose = swerveSubsystem.getPose();


        //Calculated angle to rotate to
        double angle = Math.atan2((goalPose.getY()) - robotPose.getY(), goalPose.getX()-robotPose.getX());

        //Rotation PID Calculations
        rotationPID.setGoal(angle + Math.PI - Math.toRadians(3.0));
        odometryRotation = rotationPID.calculate(robotPose.getRotation().getRadians());




        //Actual Execute
        swerveSubsystem.drive(
                forward.getAsDouble(),
                sideways.getAsDouble(),
                odometryRotation,
                true,
                true
        );



    }

    @Override
    public void initialize() {
        rotationPID.reset(swerveSubsystem.getPose().getRotation().getRadians());
    }

    @Override
    public void end(boolean interrupted) {

    }
}
