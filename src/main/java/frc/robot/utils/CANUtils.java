package frc.robot.utils;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class CANUtils {

    public static CANSparkMax configure(CANSparkMax motor) {
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 5000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 5000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 5000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 5000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 5000);
        return motor;
    }

    public static CANSparkFlex configure(CANSparkFlex motor) {
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 5000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 5000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 5000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 5000);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 5000);
        return motor;
    }
}
