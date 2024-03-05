package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.commands.DriveCommands;


public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax leftSide = new CANSparkMax(ClimberConstants.leftSideID, CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax rightSide = new CANSparkMax(ClimberConstants.rightSideID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder leftSideEncoder = leftSide.getEncoder();
    private final RelativeEncoder rightSideEncoder = rightSide.getEncoder();

    public RelativeEncoder getLeftEncoder() {
        return leftSideEncoder;
    }

    public RelativeEncoder getRightEncoder() {
        return rightSideEncoder;
    }

    /**
     * Extends the climber
     */
    public void extend() {
        rightSide.set(0.1);
        leftSide.set(0.1);
    }

    /**
     * Retracts the climber back down
     */
    public void retract() {
        rightSide.set(-0.1);
        leftSide.set(-0.1);
    }


    /**
     * Hold the motors (break mdoe)
     */
    public void hold() {
        leftSide.set(0.0);
        rightSide.set(0.0);
        leftSide.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightSide.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

}