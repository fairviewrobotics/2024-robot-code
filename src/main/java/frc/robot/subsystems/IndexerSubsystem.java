package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.utils.CANUtils;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTableUtils;

public class IndexerSubsystem extends SubsystemBase {
    private final CANSparkMax topWheel = CANUtils.configure(new CANSparkMax(IndexerConstants.topMotorID, CANSparkLowLevel.MotorType.kBrushless));


    private final CANSparkMax bottomWheels = CANUtils.configure(new CANSparkMax(IndexerConstants.bottomMotorID, CANSparkLowLevel.MotorType.kBrushless));

    private final CANSparkMax indexerRotate = CANUtils.configure(new CANSparkMax(IndexerConstants.indexerRotateID, CANSparkLowLevel.MotorType.kBrushless));

    private final AbsoluteEncoder indexerEncoder = indexerRotate.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    private final DigitalInput centerLinebreak = new DigitalInput(IndexerConstants.centerLimebreakID);



    private final BooleanEntry centerLinebreaknt = NetworkTableInstance.getDefault()
            .getTable("Indexer").getBooleanTopic("Center Linebreak").getEntry(!centerLinebreak.get());

    private Rotation2d indexerPosFirst = new Rotation2d(indexerEncoder.getPosition());
    private double indexerPosRadians = indexerPosFirst.minus(new Rotation2d(IndexerConstants.posOffset)).getRadians();



    private final ProfiledPIDController indexerPID = new ProfiledPIDController(
            IndexerConstants.indexerP,
            IndexerConstants.indexerI,
            IndexerConstants.indexerD,
            IndexerConstants.indexerTrapezoidProfile
    );

    /**
     * Indexer subsystem for everything indexer related
     */

    public IndexerSubsystem() {
        indexerPID.enableContinuousInput(-Math.PI, Math.PI);
        indexerPID.setTolerance(0.02);
        indexerEncoder.setPositionConversionFactor((2.0 * Math.PI)/23 * 16); //use constants
        indexerEncoder.setVelocityConversionFactor((2.0 * Math.PI)/ 60.0); //use constants
        indexerEncoder.setInverted(true);
        indexerRotate.setInverted(false);

//        indexerRotate.setSmartCurrentLimit(1);


    }


    /**
     * rotateMotorVolts rotates one of the motors by setting the voltage applied to the motor
     * @param motor The target motor, can be TOP_WHEEL, BOTTOM_WHEELS, or INDEXER_POS
     * @param volts The target voltage.
     */
    public void rotateMotorVolts(IndexerMotors motor, double volts) {
        switch (motor) {

            case TOP_WHEEL -> topWheel.setVoltage(volts);
            case BOTTOM_WHEELS -> bottomWheels.setVoltage(volts);
            case INDEXER_ROTATE -> indexerRotate.setVoltage(volts);
        }
    }

    /**
     * Rotate one of the motors with a percent value
     * @param motor The target motor.
     * @param percent Percent of motor speed (0.0-1.0)
     */

    public void rotateMotorPercent(IndexerMotors motor, double percent) {
        switch (motor) {
            case TOP_WHEEL -> topWheel.set(percent);
            case BOTTOM_WHEELS -> bottomWheels.set(percent);
            case INDEXER_ROTATE -> indexerRotate.set(percent);
        }
    }

    /**
     * Rotates all wheels with a percent value
     * @param percent Percent of motor speed to rotate
     */
    public void rotateAllWheelsPercent(double percent) {
        rotateMotorPercent(IndexerMotors.TOP_WHEEL, percent);
        rotateMotorPercent(IndexerMotors.BOTTOM_WHEELS, percent);
    }



    /**
     * Rotate all indexer motors with a certain voltage
     * @param volts Volts to rotate motor with
     */
    public void rotateAllWheelsVolts(double volts) {
        rotateMotorVolts(IndexerMotors.TOP_WHEEL, volts);
        rotateMotorVolts(IndexerMotors.BOTTOM_WHEELS, volts);
    }

    /**
     * Rotate the indexer to a certain position
     * @param angle The target angle for the indexer
     */
    public void moveIndexerToPos(double angle) {
        angle = MathUtils.inRange(angle, IndexerConstants.indexerMinAngle, IndexerConstants.indexerMaxAngle);
        rotateMotorVolts(IndexerMotors.INDEXER_ROTATE,
                indexerPID.calculate(indexerPosRadians, angle) +
                        IndexerConstants.indexerFF.calculate(indexerPosRadians, 0.0));
        //System.out.println(indexerEncoder.getPosition());
    }
 

    /**
     * Check if the indexer is at its goal
     * @return If the indexer is at its goal
     */
    public boolean isIndexerRotated() {
        return indexerPID.atGoal();
    }

    /**
     * Check if center limebreak is seeing something
     * @return If the limebreak is seeing something
     */
    public boolean isCenter() {
        return !centerLinebreak.get();
    }

    /**
     * Check if top limebreak is seeing something
     * @return If the limebreak is seeing something
     */
//    public boolean isTop() {
//        return !topLinebreak.get();
//    }

    /**
     * Get the current angle of the indexer
     * @return The current angle of the indexer
     */
    public double getIndexerAngle() {
        double initPos = this.indexerEncoder.getPosition();
//        System.out.println(initPos + "initPos\n-------");
//        if (indexerEncoder.getPosition() > Math.PI && indexerEncoder.getPosition() < Math.PI * 2) { initPos -= Math.PI * 2; }
//        if (initPos >= 5.75) { initPos = 0; }
        return initPos;
    }
    /**
     * Enum of possible motors to control
     */
    public enum IndexerMotors {
        TOP_WHEEL,
        BOTTOM_WHEELS,
        INDEXER_ROTATE
    }

    @Override
    public void periodic() {
//        indexerPosFirst = new Rotation2d(indexerEncoder.getPosition());
//        indexerPosRadians = indexerPosFirst.minus(new Rotation2d(IndexerConstants.posOffset)).getRadians();
//
////        System.out.println("----------------------------------------------");
////        System.out.println("Current indexer position: " + getIndexerAngle());
////        System.out.println("Current indexer goal: " + indexerPID.getGoal().position);
////        System.out.println("Current indexer error: " + indexerPID.getPositionError());
////        System.out.println("----------------------------------------------");
//
//        nt.setEntry("Top Linebreak", isTop());
//        nt.setEntry("Center Linebreak", isCenter());

        centerLinebreaknt.set(!centerLinebreak.get());
    }
}
