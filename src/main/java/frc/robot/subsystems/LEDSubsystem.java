package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
   private final Spark blinkinDriver = new Spark(9);

    public LEDSubsystem() {
        super();
    }
    @Override
    public void periodic() {
        super.periodic();
    }
    public void setLED(double value) {
        blinkinDriver.set(value);
    }
}
