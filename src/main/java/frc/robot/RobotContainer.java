// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
//  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
//
//  private final Command m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public XboxController primaryController = new XboxController(0);
  public XboxController secondaryController = new XboxController(1);
  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
//    configureButtonBindings();
    Discovery();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
//  private void configureButtonBindings() {}

  private void Discovery() {
    new JoystickButton(primaryController, XboxController.Button.kLeftBumper.value).whileHeld(new DriveCommands(
            swerveSubsystem,
            () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar / 2.0,
            () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar / 2.0,
            () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar / 2.0,
            true,
            true
    ));

    swerveSubsystem.setDefaultCommand(new DriveCommands(
            swerveSubsystem,
            () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar,
            () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar,
            () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar,
            true,
            true
    ));

    new JoystickButton(primaryController, XboxController.Button.kY.value).whileHeld(
            new RunCommand(() -> {
                    swerveSubsystem.zeroGyro();
            })
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
