package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterConstants {
    public static final double shooterP = 0.02;
    public static final double shooterI = 0.01;
    public static final double shooterD = 0.0;

    public static final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.05, 0.0179, 0.0008);



}
