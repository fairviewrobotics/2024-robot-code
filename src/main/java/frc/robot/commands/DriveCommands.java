package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommands extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;
    private final DoubleSupplier radians;
    private final boolean fieldRelativeFromButton;

    public DriveCommands(SwerveSubsystem swerveSubsystem, DoubleSupplier forward, DoubleSupplier sideways, DoubleSupplier radians, boolean fieldRelative, boolean limited) {
        this.swerveSubsystem = swerveSubsystem;
        this.forward = forward;
        this.sideways = sideways;
        this.radians = radians;
        this.fieldRelativeFromButton = true;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double forwardDesired = MathUtil.applyDeadband(forward.getAsDouble(), 0.06);
        double sidewaysDesired = MathUtil.applyDeadband(sideways.getAsDouble(), 0.06);
        double radiansDesired = MathUtil.applyDeadband(radians.getAsDouble(), 0.06);

        swerveSubsystem.drive(forwardDesired, sidewaysDesired, radiansDesired, fieldRelativeFromButton, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0.0, 0.0, 0.0, true, true);
    }



}