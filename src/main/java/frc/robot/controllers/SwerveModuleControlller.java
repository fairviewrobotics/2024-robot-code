package frc.robot.controllers;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.DrivetrainConstants;

public class SwerveModuleControlller {
    private final CANSparkMax drivingMotor;
    private final CANSparkMax turningMotor;
    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;
    private final SparkMaxPIDController drivingPID;
    private final SparkMaxPIDController turningPID;
    private SwerveModuleState m_desiredState;

    private double chassisAngularOffset;

    public SwerveModuleControlller(int drivingPort, int turningPort, double chassisAngularOffset) {

        this.chassisAngularOffset = chassisAngularOffset;
        drivingMotor = new CANSparkMax(drivingPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningPort, CANSparkMaxLowLevel.MotorType.kBrushless);
//        drivingEncoder = drivingMotor.getEncoder();
        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
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

    public SwerveModuleState getState() {
        return new SwerveModuleState(drivingEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivingEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(turningEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        drivingPID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        m_desiredState = desiredState;

    }

    public SwerveModuleState getDesiredState(){
        return m_desiredState;
    }

    public void resetEncoders() {
        drivingEncoder.setPosition(0.0);
    }
}