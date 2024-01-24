package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {

    private final ClimberSubsystem subsystem;

    private final XboxController controller;

    /**
     * Command for moving the climber up or down (this might be d@one in a weird way)
     * @param subsystem An instance of ClimberSubsystem
     * @param controller The XboxController
     * @see ClimberSubsystem Required subsystem
     */
    public ClimberCommand(ClimberSubsystem subsystem, XboxController controller) {
        this.subsystem = subsystem;
        this.controller = controller;
    }

    @Override
    public void execute() {
        if (controller.getAButtonPressed()) {
            subsystem.setState(ClimberSubsystem.ClimberState.EXTEND);
        } else {
            subsystem.setState(ClimberSubsystem.ClimberState.RETRACT);
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setState(ClimberSubsystem.ClimberState.HOLD);
    }
}
