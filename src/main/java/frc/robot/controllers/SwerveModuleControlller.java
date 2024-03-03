package frc.robot.controllers;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.DrivetrainConstants;

public class SwerveModuleControlller {
    private final CANSparkFlex drivingMotor;
    private final CANSparkMax turningMotor;
    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;
    private final SparkPIDController drivingPID;
    private final SparkPIDController turningPID;
    private SwerveModuleState m_desiredState;
    private double chassisAngularOffset;

    /**
     * This class contains all  the logic and code to control the swerve motors
     * @param drivingPort the port for the drive motor
     * @param turningPort the port for the turning motor
     * @param chassisAngularOffset the angular offset of the motor to the chassis?
     */
    public SwerveModuleControlller(int drivingPort, int turningPort, double chassisAngularOffset) {

        this.chassisAngularOffset = chassisAngularOffset;
        drivingMotor = new CANSparkFlex(drivingPort, CANSparkFlex.MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningPort, CANSparkLowLevel.MotorType.kBrushless);
//        drivingEncoder = drivingMotor.getEncoder();
        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        drivingPID = drivingMotor.getPIDController();
        turningPID = turningMotor.getPIDController();
        m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

        drivingMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        drivingPID.setFeedbackDevice(drivingEncoder);
        turningPID.setFeedbackDevice(turningEncoder);

        drivingEncoder.setPositionConversionFactor(DrivetrainConstants.drivingEncoderPositionFactor);
        drivingEncoder.setVelocityConversionFactor(DrivetrainConstants.drivingEncoderVelocityFactor);

        turningEncoder.setPositionConversionFactor(DrivetrainConstants.turningEncoderPositionFactor);
        turningEncoder.setVelocityConversionFactor(DrivetrainConstants.turningEncoderVelocityFactor);
        turningEncoder.setInverted(DrivetrainConstants.turningEncoderReversed);

        turningPID.setPositionPIDWrappingEnabled(true);
        turningPID.setPositionPIDWrappingMinInput(DrivetrainConstants.turningEncoderPositionPIDMinInput);
        turningPID.setPositionPIDWrappingMaxInput(DrivetrainConstants.turningEncoderPositionPIDMaxInput);


        drivingPID.setP(DrivetrainConstants.drivingP);
        drivingPID.setI(DrivetrainConstants.drivingI);
        drivingPID.setD(DrivetrainConstants.drivingD);
        drivingPID.setFF(DrivetrainConstants.drivingFF);
        drivingPID.setOutputRange(DrivetrainConstants.drivingMinOutput, DrivetrainConstants.drivingMaxOutput);

        turningPID.setP(DrivetrainConstants.turningP);
        turningPID.setI(DrivetrainConstants.turningI);
        turningPID.setD(DrivetrainConstants.turningD);
        turningPID.setFF(DrivetrainConstants.turningFF);
        turningPID.setOutputRange(DrivetrainConstants.turningMinOutput, DrivetrainConstants.turningMaxOutput);

        drivingMotor.setIdleMode(DrivetrainConstants.drivingMotorIdleMode);
        turningMotor.setIdleMode(DrivetrainConstants.turningMotorIdleMode);

        drivingMotor.setSmartCurrentLimit(DrivetrainConstants.drivingMotorCurrentLimit);
        turningMotor.setSmartCurrentLimit(DrivetrainConstants.turningMotorCurrentLimit);

        m_desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0.0);

        drivingMotor.burnFlash();
        turningMotor.burnFlash();
    }

    /**
     * This function gets the state of the swerve module
     * @return Returns a SwerveModuleState containing info about the velocity and rotation of the swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(drivingEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    /**
     * Gets the position of the swerve module
     * @return Returns a SwerveModulePosition containing info about the velocity and rotation of the swerve module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivingEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    /**
     * This function sets the state of a swerve module
     * @param desiredState the target state for the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(turningEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        drivingPID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
        turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkBase.ControlType.kPosition);

        m_desiredState = desiredState;

    }

    /**
     * Gets the current desired state
     * @return returns the desired state as a SwerveModuleState
     */
    public SwerveModuleState getDesiredState(){
        return m_desiredState;
    }

    /**
     * This function resets all encoders for the swerve module
     */
    public void resetEncoders() {
        drivingEncoder.setPosition(0.0);
    }
}