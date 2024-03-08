// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SendableChooser<Command> superSecretMissileTech;

  public XboxController primaryController = new XboxController(0);
  public XboxController secondaryController = new XboxController(1);
  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public IndexerSubsystem indexerSubsystem = new IndexerSubsystem();


  public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings


//FOR ALL: tune timeouts

    NamedCommands.registerCommand("AutoSpinUp", new SpinUpCommand(shooterSubsystem).withTimeout(15.0));
    NamedCommands.registerCommand("IntakeCommand", new AutoIntakeOrShoot(indexerSubsystem, intakeSubsystem, AutoIntakeOrShoot.Goal.INTAKE).withTimeout(1.5));
    NamedCommands.registerCommand("AutoSpinForShoot", new SpinUpCommand(shooterSubsystem).withTimeout(1.5));
    NamedCommands.registerCommand("AutoShoot", new AutoIntakeOrShoot(indexerSubsystem, intakeSubsystem, AutoIntakeOrShoot.Goal.SHOOT).withTimeout(1.5));
//
    superSecretMissileTech = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", superSecretMissileTech);

    // Configure the button bindings
    configureButtonBindings();
  }

  public void configureButtonBindings() {

    //DEFAULT COMMANDS

    //Test!!:
//    shooterSubsystem.setDefaultCommand(new SpinUpCommand(shooterSubsystem, indexerSubsystem));


    // PRIMARY CONTROLLER

    swerveSubsystem.setDefaultCommand(new DriveCommands(
            swerveSubsystem,
            () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar,
            () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar,
            () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar,
            true,
            true
    ));

    new JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
            new DriveCommands(swerveSubsystem,
                    () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar / 2.5,
                    () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar / 2.5,
                    () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar / 2.5,
                    true,
                    true
            )
    );

    new JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            new RunCommand(() -> swerveSubsystem.zeroGyro())
    );
//
    new JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
            new RunCommand(() -> swerveSubsystem.setX())
    );

//    new JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
//            new RunCommand(() -> intakeSubsystem.setSpeed(0.5))
//    );





    // SECONDARY CONTROLLER


//    new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
//            new IndexerCommand(indexerSubsystem, secondaryController, IndexerCommand.Mode.AMP)
//    );

//    new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
//            new ParallelCommandGroup(
//                    new RunCommand(() -> indexerSubsystem.rotateAllWheelsPercent(-0.2)),
//                    new RunCommand(() -> intakeSubsystem.setSpeed(-0.5))
//            )
//    );

//    new JoystickButton(secondaryController, XboxController.Axis.kRightTrigger.value).whileTrue(
//            new ParallelCommandGroup(
//                new SpinUpCommand(shooterSubsystem, false),
//                new IndexerCommand(indexerSubsystem, secondaryController, IndexerCommand.Mode.SPEAKER),
//                new RotateToSpeakerAndDriveCommand(swerveSubsystem,
//                        () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar / 2.0,
//                        () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar / 2.0,
//                        () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar / 2.0
//                )
//            )
//    );

//    new JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(
//            new IntakeCommand(intakeSubsystem, indexerSubsystem, IntakeCommand.Targets.AMP, true)
//    );
//     new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
//             new IntakeCommand(intakeSubsystem, indexerSubsystem, IntakeCommand.Targets.AMP, false)
//     );
//      new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
//           new IntakeCommand(intakeSubsystem, indexerSubsystem, IntakeCommand.Targets.SPEAKER, true)
//      );
//      new JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
//             new IntakeCommand(intakeSubsystem, indexerSubsystem, IntakeCommand.Targets.SPEAKER, false)
//     );
//    new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
//            new IntakeCommand(intakeSubsystem, indexerSubsystem, IntakeCommand.Targets.SPEAKER, false)
//    );
//
//    new JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
//            new RunCommand(() -> swerveSubsystem.setX())
//    );






    // SECONDARY CONTROLLER



    new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
            new SpinUpCommand(shooterSubsystem)
    );

    new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            new IntakeCommand(intakeSubsystem, indexerSubsystem, IntakeCommand.Targets.SPEAKER, false)
    );

    new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
            new RunCommand(() ->  indexerSubsystem.rotateAllWheelsPercent(0.6))
    ).whileFalse(
            new RunCommand(() -> indexerSubsystem.rotateAllWheelsPercent(0.0))
    );
    new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
            new RunCommand(() ->  indexerSubsystem.rotateAllWheelsPercent(0.15))
    ).whileFalse(
            new RunCommand(() -> indexerSubsystem.rotateAllWheelsPercent(0.0))
    );
    new JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
            new RunCommand(() ->  indexerSubsystem.rotateAllWheelsPercent(-0.15))
    ).whileFalse(
            new RunCommand(() -> indexerSubsystem.rotateAllWheelsPercent(0.0))
    );

//    new JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(
//            new ParallelCommandGroup(
//              new RunCommand(() ->  indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.BOTTOM_WHEELS, -0.6)),
//              new RunCommand(() -> indexerSubsystem.rotateMotorPercent(IndexerSubsystem.IndexerMotors.TOP_WHEEL, -0.6)),
//              new RunCommand(() -> intakeSubsystem.setTopSpeed(-0.4)),
//              new RunCommand(() -> intakeSubsystem.setTopSpeed(-0.4))
//            )
//    ).whileFalse(
//            new RunCommand(() -> indexerSubsystem.rotateAllWheelsPercent(0.0)),
//            new RunCommand(() -> int)
//    );
//    new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
//            new RunCommand(() -> indexerSubsystem.rotateAllWheelsPercent(1.0))
//    );
//    new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
//            new ParallelCommandGroup(
//              new RunCommand(() -> indexerSubsystem.rotateAllWheelsPercent(-0.3)),
//              new RunCommand(() -> intakeSubsystem.setSpeed(-0.3))
//    ));

//    new JoystickButton(secondaryController, XboxController.Axis.kLeftY.value).whileTrue(
//            new RunCommand(() -> indexerSubsystem.rotateMotorVolts(IndexerSubsystem.IndexerMotors.INDEXER_ROTATE, secondaryController.getRightY() * 2.0))
//    );
//    new JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
//            isRed ? new PathCommand(swerveSubsystem, new Pose2d(new Translation2d(1.15, 1), new Rotation2d(-120))) :
//                    new PathCommand(swerveSubsystem, new Pose2d(new Translation2d(15.4, 1), new Rotation2d(120)))
//    );
//

//    new POVButton(secondaryController, 0).whileTrue(
//            new RunCommand(() -> intakeSubsystem.setSpeed(0.5))
//    );
//
//    new POVButton(secondaryController, 180).whileTrue(
//            new RunCommand(() -> intakeSubsystem.setSpeed(-0.5))
//    );




     //TESTING COMMANDS

//    new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
//            new IntakeCommand(intakeSubsystem, indexerSubsystem, IntakeCommand.Targets.AMP, true)
//    );

//    new JoystickButton(secondaryController, XboxController.Axis.kRightTrigger.value).whileTrue(
//            new AutoRotateToSpeakerAndShoot(swerveSubsystem, indexerSubsystem)
//    );

//    new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
//            new BasicSpinUpCommand(shooterSubsystem)
//    );
//
//    new JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(
//            new RunCommand(() -> indexerSubsystem.rotateAllWheelsPercent(0.2))
//    );
//
//
//
//
//
//    new JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(
//            new AmpCommand2(indexerSubsystem, secondaryController) // This is kinda fucked
//    );
//
//    new JoystickButton(secondaryController, XboxController.Axis.kLeftTrigger.value).whileTrue(
//            new IntakeCommand(intakeSubsystem, indexerSubsystem)
//    );
//
//    new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
//            new PathCommand(swerveSubsystem)
//    );
//
//    new JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(
//            new BasicSpinUpCommand(shooterSubsystem)
//    );
//
//    new JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
//            new RunCommand(() -> shooterSubsystem.setVoltage(0.2,0.2))
//    );




  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
   // return new InstantCommand();
    return superSecretMissileTech.getSelected();
  }
}







