package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TranslateCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final double metersPerSecond;

    public TranslateCommand(SwerveSubsystem swerveSubsystem, double metersPerSecond) {
        this.swerveSubsystem = swerveSubsystem;
        this.metersPerSecond = metersPerSecond;
    }


    @Override

    public void execute() {
        swerveSubsystem.drive(metersPerSecond, 0.0, 0.0, false, true);
    }

    public double getMetersPerSecond() {
        return metersPerSecond;
    }
}
