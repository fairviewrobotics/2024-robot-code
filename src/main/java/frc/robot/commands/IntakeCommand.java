package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    private final IndexerSubsystem indexerSubsystem;

    private final LEDSubsystem ledSubsystem;

    private final Targets target;

    private final boolean source;

    private boolean finshed = false;



    private final XboxController secondaryController;


    private double rumbleTime;

    //private LEDSubsystem ledSubsystem;

    /**
     * Command to run the intake
     * @param intakeSubsystem The instance of {@link IntakeSubsystem}
     * @param indexerSubsystem The instance of {@link IndexerSubsystem} (needed for limebreak detection to stop intake motor)
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, LEDSubsystem ledSubsystem, XboxController secondaryController, Targets target, boolean source) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.target = target;
        this.source = source;
        this.secondaryController = secondaryController;

        addRequirements(intakeSubsystem, indexerSubsystem);
    }


    @Override
    public void execute() {
        if (indexerSubsystem.isCenter())
            ledSubsystem.setLED(-.07);
        else
            ledSubsystem.setLED(0.61);

        if (source)
            indexerSubsystem.moveIndexerToPos(Math.toRadians(140));
//        if (!indexerSubsystem.isCenter()) {
//

        switch (target) {
            case AMP -> {
//                if (!indexerSubsystem.isTop()) {
                    indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.TOP_WHEEL, 0.22);
                    indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.BOTTOM_WHEELS, -0.22);
                    intakeSubsystem.setTopSpeed(-0.5);
                    intakeSubsystem.setBottomSpeed(-0.8);
//                } else if (indexerSubsystem.isTop()) {
                    indexerSubsystem.rotateAllWheelsPercent(0.0);
                    intakeSubsystem.setSpeed(0.0);
//                }
            }
            case SPEAKER -> {
                if (!indexerSubsystem.isCenter()) {
                    intakeSubsystem.setTopSpeed(0.5);
                    intakeSubsystem.setBottomSpeed(0.5);
                    indexerSubsystem.rotateAllWheelsPercent(0.3);
                } else if (indexerSubsystem.isCenter()) {

                    try {
                        Thread.sleep(120);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if (indexerSubsystem.isCenter()) {
                        stopAndRumble();
                    }

                }

            }
            case FLYSHOOT -> {
                if (!indexerSubsystem.isUp()) {
                    intakeSubsystem.setTopSpeed(0.5);
                    intakeSubsystem.setBottomSpeed(0.5);
                    indexerSubsystem.rotateAllWheelsPercent(0.25);
                } else if (indexerSubsystem.isUp()) {
                    stopAndRumble();
                    finshed = true;
                }
            }
        }
    }

    private void stopAndRumble() {
        indexerSubsystem.rotateAllWheelsPercent(0);
        intakeSubsystem.setSpeed(0.0);
        double timePassed = Timer.getFPGATimestamp() - this.rumbleTime;
        System.out.println("TP: " + timePassed + " CT: " + Timer.getFPGATimestamp() + " RT: " + this.rumbleTime);
        if (timePassed >= 0.1) {
            System.out.println("Rumbling");
            secondaryController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
            this.rumbleTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void initialize() {
        rumbleTime = Timer.getFPGATimestamp();
        finshed = false;
    }

    @Override
    public void end(boolean interrupted) {
        secondaryController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        intakeSubsystem.setSpeed(0);
        indexerSubsystem.rotateAllWheelsPercent(0);
    }

    @Override
    public boolean isFinished() {
        return finshed;
    }

    public enum Targets {
        AMP,
        SPEAKER,
        FLYSHOOT
    }
}
