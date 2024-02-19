package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class VisionConstants {
    public static final double alignXP = 0.9;
    public static final double alignXI = 0;
    public static final double alignXD = 0;
    public static final TrapezoidProfile.Constraints alignXConstraints = new TrapezoidProfile.Constraints(0.75, 0.5);

    public static final double alignZP = 0.6;
    public static final double alignZI = 0;
    public static final double alignZD = 0;
    public static final TrapezoidProfile.Constraints alignZConstraints = new TrapezoidProfile.Constraints(0.75, 0.5);

    public static final double alignRotationP = 0.6;
    public static final double alignRotationI = 0;
    public static final double alignRotationD = 0;
    public static final TrapezoidProfile.Constraints alignRotationConstraints = new TrapezoidProfile.Constraints(0.75, 0.5);

    public static final double rotateToP = 0.6;
    public static final double rotateToI = 0;
    public static final double rotateToD = 0;
    public static final TrapezoidProfile.Constraints rotateToConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI/2);
}
