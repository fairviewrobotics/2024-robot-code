package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final CANSparkMax wheel1 = new CANSparkMax(IndexerConstants.wheel1ID, CANSparkLowLevel.MotorType.kBrushless);

    private final CANSparkMax wheel2 = new CANSparkMax(IndexerConstants.wheel2ID, CANSparkLowLevel.MotorType.kBrushless);

    private final CANSparkMax wheel3 = new CANSparkMax(IndexerConstants.wheel3ID, CANSparkLowLevel.MotorType.kBrushless);

    private final CANSparkMax indexerRotate = new CANSparkMax(IndexerConstants.indexerRotateID, CANSparkLowLevel.MotorType.kBrushless);

    /**
     * Set indexer pos changes the position of the indexer for either the Amp or Speaker
     * @param position Enum for the positions, can be either AMP or SPEAKER
     */
    public void setIndexerPos(IndexerPositions position) {
        switch (position) {
            case AMP -> {
                // Logic to set app position
            }
            case DOWN -> {
                // Logic to set speaker position
            }
        }
    }

    /**
     * rotateMotorVolts rotates one of the motors by setting the voltage applied to the motor
     * @param motor The target motor, can be WHEEL_1, WHEEL_2, WHEEL_3, or INDEXER_POS
     * @param volts The target voltage.
     */
    public void rotateMotorVolts(IndexerMotors motor, double volts) {
        switch (motor) {
            case WHEEL_1 -> wheel1.setVoltage(volts);
            case WHEEL_2 -> wheel2.setVoltage(volts);
            case WHEEL_3 -> wheel3.setVoltage(volts);
            case INDEXER_POS -> indexerRotate.setVoltage(volts);
        }
    }

    /**
     * Enum of the different possible positions for the indexer
     */
    public enum IndexerPositions {
        AMP,
        DOWN,
    }

    /**
     * Enum of possible motors to control
     */
    public enum IndexerMotors {
        WHEEL_1,
        WHEEL_2,
        WHEEL_3,
        INDEXER_POS
    }


}
