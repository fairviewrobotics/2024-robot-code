package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkFlex intakeMotor = new CANSparkFlex(1, CANSparkLowLevel.MotorType.kBrushless);

    /**
     * Set speed of intake
     * @param speed Target speed in percent
     */
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }
}
