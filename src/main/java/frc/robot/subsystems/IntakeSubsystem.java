package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    //Gate motor
    //Top belt
    //Bottom belt
    //Arm flippy motor

    //Two intake motors

    //Two shooter motors

    //Two climber motors

    //Defining
    private final CANSparkMax topIntakeMotor;
    private final CANSparkMax bottomIntakeMotor;
    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;


    public IntakeSubsystem() {

        topIntakeMotor = new CANSparkMax(IntakeConstants.topIntakeMotorPort, CANSparkLowLevel.MotorType.kBrushless);
        bottomIntakeMotor = new CANSparkMax(IntakeConstants.bottomIntakeMotorPort, CANSparkLowLevel.MotorType.kBrushless);

        topEncoder = topIntakeMotor.getEncoder();
        bottomEncoder = bottomIntakeMotor.getEncoder();

        topIntakeMotor.setInverted(IntakeConstants.topIntakeReversed);
        bottomIntakeMotor.setInverted(IntakeConstants.bottomIntakeReversed);

        topIntakeMotor.setSmartCurrentLimit(19);
        bottomIntakeMotor.setSmartCurrentLimit(19);

    }

    public void percentIntake(double topPercent, double bottomPercent) {
        topIntakeMotor.set(topPercent);
        bottomIntakeMotor.set(bottomPercent);
    }

    public void voltageIntake(double topVoltage, double bottomVoltage) {
        topIntakeMotor.setVoltage(topVoltage);
        bottomIntakeMotor.setVoltage(bottomVoltage);
    }

    public double topSpeed() {
        return topEncoder.getVelocity();
    }

    public double bottomSpeed() {
        return bottomEncoder.getVelocity();
    }





}
