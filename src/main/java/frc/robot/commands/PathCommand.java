package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PathCommand extends Command {

    private SwerveSubsystem swerveSubsystem;

    private double kP = 3;
    private double kI = 0;

    private double kD = 0;

    public static final double kMaxSpeedPerSecondSquared = Math.pow(DrivetrainConstants.maxSpeedMetersPerSecond, 2); // Same as above, but squared
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(DrivetrainConstants.maxSpeedMetersPerSecond, kMaxSpeedPerSecondSquared);

    public ProfiledPIDController snapController = new ProfiledPIDController(kP, kI, kD, kThetaControllerConstraints);
    public ProfiledPIDController xController = new ProfiledPIDController(kP, kI, kD, kThetaControllerConstraints);
    public ProfiledPIDController yController = new ProfiledPIDController(kP, kI, kD, kThetaControllerConstraints);

    private double x, y, angle;


    public PathCommand(SwerveSubsystem swerveSubsystem, double x, double y, double angle) {
       this.swerveSubsystem = swerveSubsystem;
       this.x = x;
       this.y = y;
       this.angle = angle;
       xController.setTolerance(0.2);
       yController.setTolerance(0.2);
       snapController.setTolerance(0.2);
       snapController.enableContinuousInput(-Math.PI, Math.PI);

       addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // The second number here is 0 because we want the robot to have no more velocity when it reaches the target
        xController.setGoal(new TrapezoidProfile.State(x, 0));
        yController.setGoal(new TrapezoidProfile.State(y, 0));
        snapController.setGoal(new TrapezoidProfile.State(angle, 0.0));

        // getPose() should return the robot's position on the field in meters, probably from odometry
        // getYaw180 just returns the reading from the gyro
        double xAdjustment = xController.calculate(swerveSubsystem.getPose().getX());
        double yAdjustment = yController.calculate(swerveSubsystem.getPose().getY());
        double angleAdjustment = snapController.calculate(swerveSubsystem.heading());

        System.out.println("\n");
        System.out.println("XError:         " + xController.getPositionError());
        System.out.println("YError:         " + yController.getPositionError());
        System.out.println("AngleError:     " + snapController.getPositionError());


        swerveSubsystem.driveRobotRelative(ChassisSpeeds.
                fromFieldRelativeSpeeds(-xAdjustment, -yAdjustment, angleAdjustment, Rotation2d.fromRadians(swerveSubsystem.heading())));

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0, 0, 0, false, false);
    }
}
