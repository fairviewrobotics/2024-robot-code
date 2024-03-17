package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.utils.CANUtils;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeTop = CANUtils.configure(new CANSparkMax(15,CANSparkLowLevel.MotorType.kBrushless));
    private  final CANSparkMax intakeBottom = CANUtils.configure(new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless));

    /**
     * Set speed of top intake wheel
     * @param speed Target speed in percent
     */
    public void setTopSpeed(double speed) {
        intakeTop.set(-speed);

    }

    /**
     * Set speed of bottom intake wheel
     * @param speed Target speed in percent
     */
    public void setBottomSpeed(double speed) {
        intakeBottom.set(-speed);
    }

    /**
     * Set speed of both intake wheels
     * @param speed Target speed in percent
     */
    public void setSpeed(double speed) {
        setTopSpeed(speed);
        setBottomSpeed(speed);

    }
}
