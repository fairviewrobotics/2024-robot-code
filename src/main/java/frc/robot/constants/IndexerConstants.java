package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class IndexerConstants {

    public final static int topMotorID = 9;
    public final static int bottomMotorID = 10;

    public final static int indexerRotateID = 11;

    public final static int centerLimebreakID = 1;
//    public final static int topLimebreakID = 1;

    public final static double indexerP = 8.0;
    public final static double indexerI = 0.0;
    public final static double indexerD = 0.0;

    public final static double posOffset = 0.0;
    public final static double indexerMinAngle = Math.toRadians(2.0);
    public final static double indexerMaxAngle = Math.toRadians(190.0);

    //public final static double indexerPositionConversionFactor = ((2.0 * Math.PI) * 23/16 * 48);
    //public final static double indexerVelocityConversionFactor = ((2.0 * Math.PI)/ 60.0);
    //public final static double maxIndexerAngle = Math.toDegrees(0.4);
    //public final static double minIndexerAngle = Math.toDegrees(-0.10);

    public final static TrapezoidProfile.Constraints indexerTrapezoidProfile =
            new TrapezoidProfile.Constraints(Math.PI * (3/2), Math.PI);

    public final static ArmFeedforward indexerFF = new ArmFeedforward(0.12, 0.458, 0.00);

}
