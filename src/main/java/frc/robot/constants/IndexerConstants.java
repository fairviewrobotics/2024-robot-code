package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class IndexerConstants {
    public final static int wheel1ID = 9;
    public final static int wheel2ID = 10;
    public final static int wheel3ID = 13;
    public final static int indexerRotateID = 14;

    public final static int centerLimebreakID = 10;
    public final static int topLimebreakID = 9;

    public final static double indexerP = 3.0;
    public final static double indexerI = 0.0;
    public final static double indexerD = 0.0;

    public final static TrapezoidProfile.Constraints indexerTrapezoidProfile =
            new TrapezoidProfile.Constraints(Math.PI * 3, Math.PI * 5);

    public final static ArmFeedforward indexerFF = new ArmFeedforward(0.13, 0.5, 0.00);

}
