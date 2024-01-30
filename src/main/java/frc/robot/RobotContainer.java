// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathCommand;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.SwerveSubsystem;

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

    new JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            new RunCommand(() -> {
              swerveSubsystem.zeroGyro();
            })
    );

    new JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(
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
