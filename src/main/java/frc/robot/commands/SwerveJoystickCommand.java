// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommand extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, rotSpdFunction;
  private final Supplier<Boolean> fieldRelativeFunction;
  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

  public static double rotSim = 0;

  /** Creates a new SwerveJoystickCommand. */
  public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem, 
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> rotSpdFunction, 
            Supplier<Boolean> fieldRelativeFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.rotSpdFunction = rotSpdFunction;
    this.fieldRelativeFunction = fieldRelativeFunction;

    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleopDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleopDriveMaxAccelerationUnitsPerSecond);
    this.rotLimiter = new SlewRateLimiter(DriveConstants.kTeleopDriveMaxAngularAccelerationUnitsPerSecond);

    addRequirements(swerveSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get joystick values
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double rotSpeed = rotSpdFunction.get();
    
    //apply deadband
    xSpeed = Math.abs(xSpeed) > OperatorConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OperatorConstants.kDeadband ? ySpeed : 0.0;
    rotSpeed = Math.abs(rotSpeed) > OperatorConstants.kDeadband ? rotSpeed : 0.0;

    //apply limiter
    xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.kMaxSpeedMetersPerSecond / 2);
    ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.kMaxSpeedMetersPerSecond / 2);
    rotSpeed = rotLimiter.calculate(rotSpeed) * DriveConstants.kMaxAngularSpeedRadiansPerSecond / 2;
    
    if (rotSpeed > 0){
      rotSim += 1;
    } else if (rotSpeed < 0){
      rotSim -= 1;
    }

    //construct chassis speed
    ChassisSpeeds chassisSpeeds;
    if (fieldRelativeFunction.get()){
       //if field relative mode (default)
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotSpeed, swerveSubsystem.getGyroRotation2d(rotSim));
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    }

    //convert chassis speed to module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //output module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
