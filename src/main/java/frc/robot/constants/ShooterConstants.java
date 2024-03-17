package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.VisionUtils;

public class ShooterConstants {

    public static final int shooterTopMotorID = 13;
    public static final int shooterBottomMotorID = 14;

//    public static final double shooterP = 0.02;
    public static final double shooterP = 0.0;
    public static final double shooterI = 0.0;
    public static final double shooterD = 0.0;

    public static final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.25, 0.0175, 0.0);

    public static final PIDController shooterPID = new PIDController(ShooterConstants.shooterP,ShooterConstants.shooterI, ShooterConstants.shooterD);
    public static boolean isActive = false;
    public static double rotateP = 2.6;
    public static double rotateI = 0.0;
    public static double rotateD = 0.2;
    public static final TrapezoidProfile.Constraints rotateConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI/2);

    public static final double shooterRPM = 6500;
    public static final double shooterWheelDiameterInches = 4.0;
    public static final double shooterWheelDiameterMeters = shooterWheelDiameterInches/39.37;
    public static final double shooterAngle = Math.toRadians(37);
    public static final double shooterNoteSpeedMPS = 2 * Math.PI * (shooterWheelDiameterMeters/2) * shooterRPM / 60;
    public static final double shooterNoteSpeedX = shooterNoteSpeedMPS * Math.cos(shooterAngle);
    public static final double shooterNoteSpeedY = shooterNoteSpeedMPS * Math.sin(shooterAngle);

    public static final double speakerHeight = 1.98;
    public static final double shooterHeight = 0.46;
    public static final double timeToSpeakerHeight = speakerHeight/shooterNoteSpeedY;

    public static final double timeToSpeakerHeightWithGravity = (
            (
                    -shooterNoteSpeedY
                            + Math.sqrt(
                            Math.pow(shooterNoteSpeedY,2)
                                    - (4 * (-4.9) * (-(speakerHeight-shooterHeight)))
                    )
            ) / (2 * (-4.9))
    );



    public static final double shootDelayTime = 0.34;

//    public static final Pose2d speakerPoseRed = new Pose2d(VisionConstants.fieldLenMeters, VisionConstants.fieldHighMeters - 2.5, Rotation2d.fromRadians(0.0));
    public static final Pose2d speakerPoseRed = new Pose2d(16.5, 5.5, Rotation2d.fromRadians(0.0));

    public static final Pose2d speakerPoseBlue = new Pose2d(0.0,5.5, Rotation2d.fromRadians(0.0));
//    public static final Pose2d speakerPoseRed = new Pose2d(0.0,5.5, Rotation2d.fromRadians(0.0));







}
