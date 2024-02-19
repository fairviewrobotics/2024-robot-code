package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;


import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendClimberCommand extends Command {
    private final ClimberSubsystem climberSubsystem;

    private RelativeEncoder leftEncoder;

//    private RelativeEncoder rightEncoder;

    double leftEncoderPos;
//    double rightEncoderPos;

    public ExtendClimberCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.leftEncoder = climberSubsystem.getLeftEncoder();
//        this.rightEncoder = climberSubsystem.getRightEncoder();
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {

        leftEncoderPos = leftEncoder.getPosition();
//        rightEncoderPos = rightEncoder.getPosition();

        if (leftEncoderPos > 3.0) {
            climberSubsystem.hold();
        } else {
            climberSubsystem.extend();
        }

    }

}
