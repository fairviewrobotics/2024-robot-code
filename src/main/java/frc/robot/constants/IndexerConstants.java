package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class IndexerConstants {
    public final static int topMotorID = 10;
    public final static int bottomMotorID = 9;
    public final static int indexerRotateID = 11;

    public final static int centerLimebreakID = 0;
    public final static int topLimebreakID = 1;

    public final static double indexerP = 3.0;
    public final static double indexerI = 0.0;
    public final static double indexerD = 0.0;

    public final static TrapezoidProfile.Constraints indexerTrapezoidProfile =
            new TrapezoidProfile.Constraints(Math.PI * 3, Math.PI * 5);

    public final static ArmFeedforward indexerFF = new ArmFeedforward(0.13, 0.5, 0.00);

}
