package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class IndexerConstants {

    public final static int wheel1ID = 9;
    public final static int wheel2ID = 10;

    public final static int indexerRotateID = 14;

    public final static int centerLimebreakID = 1;
    public final static int topLimebreakID = 0;

    public final static double indexerP = 8.0;
    public final static double indexerI = 0.0;
    public final static double indexerD = 0.0;

    //public final static double maxIndexerAngle = Math.toDegrees(0.4);
    //public final static double minIndexerAngle = Math.toDegrees(-0.10);

    public final static TrapezoidProfile.Constraints indexerTrapezoidProfile =
            new TrapezoidProfile.Constraints(Math.PI * (3/2), Math.PI);

    public final static ArmFeedforward indexerFF = new ArmFeedforward(0.12, 0.458, 0.00);

}
