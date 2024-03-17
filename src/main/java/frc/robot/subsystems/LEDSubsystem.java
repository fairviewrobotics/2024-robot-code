package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    Spark blinkinDriver = new Spark(0);

    /**
     * Sets the color of the LEDs on the robot
     * @param value The color of LEDs between -1 and 1. (<a href="https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf">Reference</a>)
     */
    public void setLED(double value) {
        blinkinDriver.set(value);
    }
}