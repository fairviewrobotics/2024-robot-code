// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberTestCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

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
//  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

//  public ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
//    configureButtonBindings();
    configureButtonBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void configureButtonBindings(){

//    swerveSubsystem.setDefaultCommand(new DriveCommands(
//            swerveSubsystem,
//            () -> primaryController.getLeftY() * DrivetrainConstants.drivingSpeedScalar / 4.0,
//            () -> primaryController.getLeftX() * DrivetrainConstants.drivingSpeedScalar / 4.0,
//            () -> primaryController.getRightX() * DrivetrainConstants.rotationSpeedScalar / 4.0,
//            true,
//            true
//    ));
//
//    new JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
//            new RunCommand(() -> {
//              swerveSubsystem.zeroGyro();
//            })
//    );

//    new JoystickButton(primaryController, XboxController.Axis.kRightTrigger.value).whileTrue(
//            new ClimberTestCommand((XboxController.Axis.kRightTrigger.value/5), climberSubsystem)
//    );
//
//    new JoystickButton(primaryController, XboxController.Axis.kLeftTrigger.value).whileTrue(
//            new ClimberTestCommand((-1 * (XboxController.Axis.kLeftTrigger.value/5)), climberSubsystem)
//    );
//
//    new JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
//            new ClimberTestCommand(-0.5, climberSubsystem)
//    );
//
//    new JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(
//            new ClimberTestCommand(0.5, climberSubsystem)
//    );
//    new JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
//            new ClimberTestCommand(-0.2, climberSubsystem)
//    );
//
//    new JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
//            new ClimberTestCommand(0.2, climberSubsystem)
//    );

    new JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(
            new ShooterCommands(shooterSubsystem, -0.75, -0.75)
    );

    new JoystickButton(primaryController, XboxController.Axis.kRightTrigger.value).whileTrue(
            new ShooterCommands(shooterSubsystem, primaryController.getRightTriggerAxis() * -1, primaryController.getRightTriggerAxis() * -1)
    );





  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }
}
