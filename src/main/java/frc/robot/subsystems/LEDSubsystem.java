package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    Spark BlinkinDriver = new Spark(9);
    DoublePublisher LED_NT = NetworkTableInstance.getDefault().getTable("LEDs").getDoubleTopic("LEDValue").publish();
    @Override
    public void periodic() {
        super.periodic();
        LED_NT.set(BlinkinDriver.get());
    }
    public void setLED(double value) {
        BlinkinDriver.set(value);
    }
}