// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.constants.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public XboxController primaryController = new XboxController(0);
  public XboxController secondaryController = new XboxController(1);
  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public void configureButtonBindings(){

    swerveSubsystem.setDefaultCommand(new DriveCommands(
            swerveSubsystem,
            () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar / 2.0,
            () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar / 2.0,
            () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar / 2.0,
            true,
            true
    ));

    indexerSubsystem.setDefaultCommand(new BaseCommand(indexerSubsystem));

    // Primary controller
    new JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            new RunCommand(() -> swerveSubsystem.zeroGyro())
    );

    // Secondary controller
    new JoystickButton(secondaryController, XboxController.Axis.kRightTrigger.value).whileTrue(
            new SpinUpCommand(shooterSubsystem)
    );

    new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
            new SpeakerCommand(indexerSubsystem, swerveSubsystem.getFieldRelativeChassisSpeeds(), swerveSubsystem.getPose())
    );

    new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            new AmpCommand(indexerSubsystem, secondaryController)
    );

    new JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(
            new AmpCommand2(indexerSubsystem, secondaryController) // This is kinda fucked
    );

    new JoystickButton(secondaryController, XboxController.Axis.kLeftTrigger.value).whileTrue(
            new IntakeCommand(intakeSubsystem, indexerSubsystem)
    );

    new JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
            new PathCommand(swerveSubsystem)
    );


  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
