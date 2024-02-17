package frc.robot.commands;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RetractClimberCommand extends Command {
    private final ClimberSubsystem climberSubsystem;

    private final RelativeEncoder leftEncoder;

//    private RelativeEncoder rightEncoder;


    public RetractClimberCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.leftEncoder = climberSubsystem.getLeftEncoder();
//        this.rightEncoder = climberSubsystem.getRightEncoder();
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        double leftEncoderPos = leftEncoder.getPosition();
//        double rightEncoderPos = rightEncoder.getPosition();

        climberSubsystem.retract();

        if (leftEncoderPos < 1.0) {
            climberSubsystem.hold();
        } else {
            climberSubsystem.retract();
        }

    }
}
