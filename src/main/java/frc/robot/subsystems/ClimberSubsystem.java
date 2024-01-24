package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax leftSide = new CANSparkMax(ClimberConstants.leftSideID, CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax rightSide = new CANSparkMax(ClimberConstants.leftSideID, CANSparkLowLevel.MotorType.kBrushless);

    private ClimberState state;
    /**
     * This function sets the state of the climber, possible options are EXTEND, RETRACT, or HOLD
     * @param state The state we are setting the climber to.
     */
    public void setState(ClimberState state) {
        this.state = state;
    }

    /**
     * Gets the current state of the climber
     * @return Returns the state of the climber as a ClimberState
     */
    public ClimberState getState() {
        return this.state;
    }

    @Override
    public void periodic() {
        switch (this.state) {
            case EXTEND -> {
                // Extend logic
            }
            case RETRACT -> {
                // Retract logic
            }
            case HOLD -> {
                // Hold logic
                leftSide.setIdleMode(CANSparkBase.IdleMode.kBrake);
                rightSide.setIdleMode(CANSparkBase.IdleMode.kBrake);
            }
        }
    }

    /**
     * Enum of the 3 different climber states
     */
    public enum ClimberState {
        EXTEND,
        RETRACT,
        HOLD
    }
}
