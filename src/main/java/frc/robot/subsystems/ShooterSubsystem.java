package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.NetworkTableUtils;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkFlex topShooterMotor;
    private final CANSparkFlex bottomShooterMotor;
    private final RelativeEncoder topShooterEncoder;
    private final RelativeEncoder bottomShooterEncoder;

    private final NetworkTableUtils shooterTable;


    private final DoubleEntry shooterRPMTop;
    private final DoubleEntry shooterRPMBottom;

    //private MotorControllerGroup shooterMotorGroup;

    public ShooterSubsystem() {
        
        topShooterMotor = new CANSparkFlex(ShooterConstants.topShooterMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        bottomShooterMotor = new CANSparkFlex(ShooterConstants.bottomShooterMotorPort, CANSparkLowLevel.MotorType.kBrushless);

        topShooterEncoder = topShooterMotor.getEncoder();
        bottomShooterEncoder = bottomShooterMotor.getEncoder();

        topShooterMotor.setInverted(ShooterConstants.topShooterReversed);
        bottomShooterMotor.setInverted(ShooterConstants.bottomShooterReversed);

        topShooterMotor.setSmartCurrentLimit(38);
        bottomShooterMotor.setSmartCurrentLimit(38);

        shooterTable = new NetworkTableUtils("Shooter");

        shooterRPMTop = NetworkTableInstance.getDefault().getTable("Shooter").getDoubleTopic("RPMTop").getEntry(0.0);
        shooterRPMBottom = NetworkTableInstance.getDefault().getTable("Shooter").getDoubleTopic("RPMBottom").getEntry(0.0);




    }

    public void percentShoot(double topPercent, double bottomPercent) {
        topShooterMotor.set(topPercent);
        bottomShooterMotor.set(bottomPercent);
        shooterRPMTop.set(topShooterEncoder.getVelocity());
        shooterRPMBottom.set(bottomShooterEncoder.getVelocity());

        System.out.println(topShooterEncoder.getVelocity());
    }

    public void voltageShoot(double topVoltage, double bottomVoltage) {
        topShooterMotor.setVoltage(topVoltage);
        bottomShooterMotor.setVoltage(bottomVoltage);
    }

    public double topShooterSpeed() {
        return topShooterEncoder.getVelocity();
    }

    public double bottomShooterSpeed() {
        return bottomShooterEncoder.getVelocity();
    }
}