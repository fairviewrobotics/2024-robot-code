package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.MathUtils;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.shooterTopMotorID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.shooterBottomMotorID, CANSparkLowLevel.MotorType.kBrushless);

    private final PIDController shooterPID = new PIDController(ShooterConstants.shooterP,ShooterConstants.shooterI, ShooterConstants.shooterD);

    private final DoubleEntry ShooterSetpoint = NetworkTableInstance.getDefault()
            .getTable("Shooter").getDoubleTopic("Setpoint").getEntry(0.0);

    private final DoubleEntry ShooterSpeedTop = NetworkTableInstance.getDefault()
            .getTable("Shooter").getDoubleTopic("Top").getEntry(0.0);

    private final DoubleEntry ShooterSpeedBottom = NetworkTableInstance.getDefault()
            .getTable("Shooter").getDoubleTopic("Bottom").getEntry(0.0);

    public ShooterSubsystem(){
        topMotor.setInverted(true);
        bottomMotor.setInverted(true);
        topMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        bottomMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    }


    /**
     * Set the target speed for the shooter
     * @param speed target speed in rpm
     */
    public void setSpeed(double speed) {
        topMotor.setVoltage(shooterPID.calculate(MathUtils.rpmToRadians(topMotor.getEncoder().getVelocity()), MathUtils.rpmToRadians(speed)) +
                ShooterConstants.shooterFF.calculate(MathUtils.rpmToRadians(speed)));

        bottomMotor.setVoltage(shooterPID.calculate(MathUtils.rpmToRadians(bottomMotor.getEncoder().getVelocity()), MathUtils.rpmToRadians(speed)) +
                ShooterConstants.shooterFF.calculate(MathUtils.rpmToRadians(speed)));

        ShooterSetpoint.set(speed);
    }

    public double getTopMotorRPM(){
        return topMotor.getEncoder().getVelocity();
    }

    public double getBottomMotorRPM(){
        return bottomMotor.getEncoder().getVelocity();
    }

    public void setVoltage(double v1, double v2) {
        topMotor.setVoltage(v1);
        bottomMotor.setVoltage(v2);
    }

    @Override
    public void periodic() {
        ShooterSpeedBottom.set(getBottomMotorRPM());
        ShooterSpeedTop.set(getTopMotorRPM());
    }

}
