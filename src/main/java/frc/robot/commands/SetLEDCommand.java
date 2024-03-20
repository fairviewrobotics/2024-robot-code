package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDCommand extends Command {
    private final LEDSubsystem subsystem;

    private final LEDConstants.Status status;
    public SetLEDCommand(LEDSubsystem subsystem, LEDConstants.Status status) {
        this.status = status;

        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
    }


}
