package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTestCommand extends Command {

    public final ClimberSubsystem climberSubsystem;

    public final double motorSpeed;

    public ClimberTestCommand(double motorSpeed, ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.motorSpeed = motorSpeed;

        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.setLock(true);
        climberSubsystem.setSpeed(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setSpeed(0.0);
    }
}
