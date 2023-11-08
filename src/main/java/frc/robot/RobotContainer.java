// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private final PS4Controller Pilot = new PS4Controller(0);
     
  public RobotContainer() {
    SmartDashboard.putNumber("Left Y", 0.0);
    SmartDashboard.putNumber("Left X", 0.0);
    SmartDashboard.putNumber("Right X", 0.0);

    m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
      m_swerveSubsystem, 
      () -> SmartDashboard.getNumber("Left Y", 0),
      () -> -SmartDashboard.getNumber("Left X", 0),
      () -> -SmartDashboard.getNumber("Right X", 0),
      () -> !Pilot.getR1Button()));

    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
    new Trigger(() -> Pilot.getOptionsButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));
    
    new Trigger(() -> Pilot.getCrossButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.brakeModules()));
    new Trigger(() -> Pilot.getCircleButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.coastMotors()));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
