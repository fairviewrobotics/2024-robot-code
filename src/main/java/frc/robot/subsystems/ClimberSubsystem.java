package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax leftSide = new CANSparkMax(ClimberConstants.leftSideID, CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax rightSide = new CANSparkMax(ClimberConstants.leftSideID, CANSparkLowLevel.MotorType.kBrushless);

    /**
     * This function sets the state of the climber, possible options are EXTEND, RETRACT, or HOLD
     * @param state The state we are setting the climber to.
     */
    public void setClimberState(ClimberStates state) {
        switch (state) {
            case EXTEND -> {
                // Extend logic
            }
            case RETRACT -> {
                // Retract logic
            }
            case HOLD -> {
                // Hold logic
            }
        }
    }

    /**
     * Enum of the 3 different climber states
     */
    public enum ClimberStates {
        EXTEND,
        RETRACT,
        HOLD
    }
}
