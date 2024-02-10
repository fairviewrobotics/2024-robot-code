package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDCommand extends Command {
    private static LEDSubsystem subsystem;
    public SetLEDCommand(LEDSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    //@Override
    public static void run(LEDConstants.Status currentStatus) {
        switch(currentStatus) {
            case TEST_1:
                //Confetti
                subsystem.setLED(-0.87);
            case TEST_2:
                //Sinelon, Forest Palette
                subsystem.setLED(-0.71);
        }
    }
}
