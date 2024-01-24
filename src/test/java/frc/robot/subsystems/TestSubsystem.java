package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class TestSubsystem extends SubsystemBase {

    @Test
    public void pass() {
        int x = 6;
        int y = 4;
        assertEquals(10, x + y);
    }

    @Test
    public void fail() {
        int x = 5;
        int y = 4;
        assertEquals(10, x + y);
    }
}
