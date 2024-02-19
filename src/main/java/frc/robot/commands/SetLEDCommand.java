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
        switch(status) {
//            case TEST_1:
//                //Confetti
//                subsystem.setLED(-0.87);
//            case TEST_2:
//                //Sinelon, Forest Palette
//                subsystem.setLED(-0.71);
            case IDLE:
                //Solid, Gold
                subsystem.setLED(0.69);
            case INTAKING:
                //Solid, Aqua
                subsystem.setLED(0.81);
            case MOVING:
                //Solid, White
                subsystem.setLED(0.93);
            case DISABLED:
                //Solid, Red
                subsystem.setLED(0.61);
            case SHOOTING:
                //Solid, Lime
                subsystem.setLED(0.73);
            case VISION_MOVING:
                //Solid, Blue
                subsystem.setLED(0.87);
            case SPINUP:
                //Solid, Violet
                subsystem.setLED(0.91);
            case CLIMBER_EXTENDING:
                //Shot, Blue
                subsystem.setLED(-0.83);
            case CLIMBER_RETRACTING:
                //Shot, Red
                subsystem.setLED(-0.85);
        }
    }

}
