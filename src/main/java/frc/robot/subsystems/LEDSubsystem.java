package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    Spark blinkinDriver = new Spark(9);

    /**
     * Sets the color of the LEDs on the robot
     * @param value The color of LEDs (between 0-1, no clue how to figure out)
     */
    public void setLED(double value) {
        blinkinDriver.set(value);
    }
}