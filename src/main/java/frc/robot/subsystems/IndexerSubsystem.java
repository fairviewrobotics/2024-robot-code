package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final CANSparkMax wheel1 = new CANSparkMax(IndexerConstants.wheel1ID, CANSparkLowLevel.MotorType.kBrushless);


    private final CANSparkMax wheel2 = new CANSparkMax(IndexerConstants.wheel2ID, CANSparkLowLevel.MotorType.kBrushless);

    private final CANSparkMax wheel3 = new CANSparkMax(IndexerConstants.wheel3ID, CANSparkLowLevel.MotorType.kBrushless);

    private final CANSparkMax indexerRotate = new CANSparkMax(IndexerConstants.indexerRotateID, CANSparkLowLevel.MotorType.kBrushless);

    private final AbsoluteEncoder indexerEncoder = indexerRotate.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    private final DigitalInput centerLimebreak = new DigitalInput(IndexerConstants.centerLimebreakID);

    private final DigitalInput topLimebreak = new DigitalInput(IndexerConstants.topLimebreakID);

    private final ProfiledPIDController indexerPID = new ProfiledPIDController(
            IndexerConstants.indexerP,
            IndexerConstants.indexerI,
            IndexerConstants.indexerD,
            IndexerConstants.indexerTrapezoidProfile
    );

    public IndexerSubsystem() {
        indexerPID.setTolerance(0.2);
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

    public void rotateMotorPercent(IndexerMotors motor, double percent) {
        switch (motor) {
            case WHEEL_1 -> wheel1.set(percent);
            case WHEEL_2 -> wheel2.set(percent);
            case WHEEL_3 -> wheel3.set(percent);
            case INDEXER_POS -> indexerRotate.set(percent);
        }
    }

    public void rotateAllWheels(double percent) {
        rotateMotorPercent(IndexerMotors.WHEEL_1, percent);
        rotateMotorPercent(IndexerMotors.WHEEL_2, percent);
        rotateMotorPercent(IndexerMotors.WHEEL_3, percent);

    }


    public void moveIndexerToPos(double angle) {
        rotateMotorVolts(IndexerMotors.INDEXER_POS,
                indexerPID.calculate(indexerEncoder.getPosition(), angle) + IndexerConstants.indexerFF
                        .calculate(indexerEncoder.getPosition(), 0.0));
    }

    public boolean isIndexerRotated() {
        return indexerPID.atGoal();
    }

    public boolean getCenterLimebreak() {
        return this.centerLimebreak.get();
    }

    public boolean getTopLimebreak() {
        return this.topLimebreak.get();
    }

    public double getIndexerAngle() {
        return this.indexerEncoder.getPosition();
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