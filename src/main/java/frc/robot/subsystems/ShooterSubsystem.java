package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.MathUtils;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkFlex topMotor = new CANSparkFlex(1, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex bottomMotor = new CANSparkFlex(1, CANSparkLowLevel.MotorType.kBrushless);

    private final PIDController shooterPID = new PIDController(ShooterConstants.shooterP,ShooterConstants.shooterI, ShooterConstants.shooterD);

    /**
     * Set the target speed for the shooter
     * @param speed target speed in rpm
     */
    public void setSpeed(double speed) {
        topMotor.setVoltage(shooterPID.calculate(MathUtils.rpmToRadians(topMotor.getEncoder().getVelocity()), MathUtils.rpmToRadians(speed)) +
                ShooterConstants.shooterFF.calculate(MathUtils.rpmToRadians(speed)));

        bottomMotor.setVoltage(shooterPID.calculate(MathUtils.rpmToRadians(bottomMotor.getEncoder().getVelocity()), MathUtils.rpmToRadians(speed)) +
                ShooterConstants.shooterFF.calculate(MathUtils.rpmToRadians(speed)));
    }

}
