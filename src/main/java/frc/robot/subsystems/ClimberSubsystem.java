package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax climberMotor;

    public ClimberSubsystem() {
        this.climberMotor = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void setLock(boolean locked) {
        if (locked) climberMotor.setIdleMode(CANSparkBase.IdleMode.kBrake); else climberMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    }

}
