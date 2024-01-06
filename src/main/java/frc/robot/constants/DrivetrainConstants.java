package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {

    public static final double maxSpeedMetersPerSecond = 4.0;
    public static final double maxAngularSpeed = Math.PI;

    public static final double directionSlewRate = 4.0; // rads/sec - turning
    public static final double magnitudeSlewRate = 5.0; // percent/second (1 = 100%) - forward/backward/traverse
    public static final double rotationalSlewRate = 12.0; // percent/second (1 = 100%) - rotation

    public static final double drivingSpeedScalar = -1.0;
    public static final double rotationSpeedScalar = -2.0;

    public static final double trackWidth = Units.inchesToMeters(26.5);
    public static final double wheelBase = Units.inchesToMeters(26.5);

    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2)
    );

    public static final double frontLeftChassisAngularOffset = 5.772;
    public static final double frontRightChassisAngularOffset = 6.099;
    public static final double rearLeftChassisAngularOffset = 0.871 + Math.PI;
    public static final double rearRightChassisAngularOffset = 3.650;

    public static final int frontLeftDrivingPort = 5;
    public static final int rearLeftDrivingPort = 1;
    public static final int frontRightDrivingPort = 7;
    public static final int rearRightDrivingPort = 3;

    public static final int frontLeftTurningPort = 6;
    public static final int rearLeftTurningPort = 2;
    public static final int frontRightTurningPort = 8;
    public static final int rearRightTurningPort = 4;

    public static final boolean gyroReversed = false;
    public static final boolean turningEncoderReversed = true;

    public static final int drivingMotorPinionTeeth = 13;

    public static final double freeSpeedRpm = 5676.0;
    public static final double drivingMotorFreeSpeedRps = freeSpeedRpm / 60.0;
    public static final double wheelDiameterMeters = 0.0762;
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    public static final double drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
    public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters) / drivingMotorReduction;

    public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI) / drivingMotorReduction;
    public static final double drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI) / drivingMotorReduction) / 60.0;
    public static final double turningEncoderPositionFactor = 2 * Math.PI;
    public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0;
    public static final double turningEncoderPositionPIDMinInput = 0.0;
    public static final double turningEncoderPositionPIDMaxInput = turningEncoderPositionFactor;

    public static final double drivingP = 0.06;
    public static final double drivingI = 0.0;
    public static final double drivingD = 0.0;
    public static final double drivingFF = 1.0 / driveWheelFreeSpeedRps;
    public static final double drivingMinOutput = -1.0;
    public static final double drivingMaxOutput = 1.0;

    public static final double turningP = 0.6;
    public static final double turningI = 0.0;
    public static final double turningD = 0.0;
    public static final double turningFF = 0.0;
    public static final double turningMinOutput = -1.0;
    public static final double turningMaxOutput = 1.0;

    public static final CANSparkMax.IdleMode drivingMotorIdleMode = CANSparkMax.IdleMode.kCoast;
    public static final CANSparkMax.IdleMode turningMotorIdleMode = CANSparkMax.IdleMode.kBrake;

    public static final int drivingMotorCurrentLimit = 40;
    public static final int turningMotorCurrentLimit = 20;
}