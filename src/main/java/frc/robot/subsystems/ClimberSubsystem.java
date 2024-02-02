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
     * This function sets the state of the  climber, possible options are EXTEND, RETRACT, or HOLD
     * @param state The state we are setting the climber to (see {@link ClimberState}).
     */
    public void setState(ClimberState state) {
        this.state = state;
    }

    /**
     * Gets the current state of the climber
     * @return Returns the state of the climber as a {@link ClimberState}
     */
    public ClimberState getState() {
        return this.state;
    }

    @Override
    public void periodic() {
        switch (this.state) {
            case EXTEND -> {
                // Extend logic
                rightSide.setVoltage(3.0);
                leftSide.setVoltage(3.0);
            }
            case RETRACT -> {
                // Retract logic
                rightSide.setVoltage(-3.0);
                leftSide.setVoltage(-3.0);
            }
            case HOLD -> {
                // Hold logic
                leftSide.setVoltage(0.0);
                rightSide.setVoltage(0.0);
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
