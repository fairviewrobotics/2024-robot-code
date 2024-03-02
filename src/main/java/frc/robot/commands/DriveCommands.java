package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommands extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;
    private final DoubleSupplier radians;
    private final boolean fieldRelativeFromButton;

    /**
     * This class contains all the drive commands for swerve
     * @param swerveSubsystem SwerveSubsystem instance for controlling the swerve drive
     * @param forward The target forward meters/second
     * @param sideways The target sideways meters/second
     * @param radians The target radian for the rotation
     * @param fieldRelative If the swerve should be relative to the robot or the field
     * @param limited If we are limiting the motors
     */
    public DriveCommands(SwerveSubsystem swerveSubsystem, DoubleSupplier forward, DoubleSupplier sideways, DoubleSupplier radians, boolean fieldRelative, boolean limited) {
        this.swerveSubsystem = swerveSubsystem;
        this.forward = forward;
        this.sideways = sideways;
        this.radians = radians;
        this.fieldRelativeFromButton = true;
        addRequirements(swerveSubsystem);
    }

    // Don't write javadoc for wpilib functions
    @Override
    public void execute() {
        double forwardDesired = MathUtil.applyDeadband(forward.getAsDouble(), 0.03);
        double sidewaysDesired = MathUtil.applyDeadband(sideways.getAsDouble(), 0.03);
        double radiansDesired = MathUtil.applyDeadband(radians.getAsDouble(), 0.03);

        swerveSubsystem.drive(-forwardDesired, -sidewaysDesired, radiansDesired, fieldRelativeFromButton, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0.0, 0.0, 0.0, true, true);
    }



}