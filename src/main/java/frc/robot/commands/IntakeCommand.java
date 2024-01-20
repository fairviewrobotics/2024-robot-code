package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    public final IntakeSubsystem intakeSubsystem;
    public final double topPercent;
    public final double bottomPercent;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double topPercent, double bottomPercent) {
        this.intakeSubsystem = intakeSubsystem;
        this.topPercent = topPercent;
        this.bottomPercent = bottomPercent;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.percentIntake(topPercent, bottomPercent);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.voltageIntake(0.0, 0.0);
    }

}
