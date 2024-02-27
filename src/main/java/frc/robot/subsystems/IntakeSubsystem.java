package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeTop = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
    private  final CANSparkMax intakeBottom = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);

    /**
     * Set speed of top intake wheel
     * @param speed1 Target speed in percent
     */
    public void setTopSpeed(double speed1) {
        intakeTop.set(speed1);

    }

    /**
     * Set speed of bottom intake wheel
     * @param speed2 Target speed in percent
     */
    public void setBottomSpeed(double speed2) {
        intakeBottom.set(speed2 * -1.0);
    }

    /**
     * Set speed of both intake wheels
     * @param speed3 Target speed in percent
     */
    public void setSpeed(double speed3) {
        setTopSpeed(speed3);
        setBottomSpeed(speed3 * -1.0);

    }
}
