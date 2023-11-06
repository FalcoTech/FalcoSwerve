// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommand extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, rotSpdFunction;
  private final Supplier<Boolean> fieldRelativeFunction;
  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

  private final Timer idleTimer = new Timer();
  private boolean isRobotIdle = false;

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
    double xSpeed = xSpdFunction.get(); //get X speed from function
    double ySpeed = ySpdFunction.get(); //get Y speed from function
    double rotSpeed = rotSpdFunction.get(); //get rotation speed from function

    if (Math.abs(xSpeed) < OperatorConstants.kPilotDeadband && Math.abs(ySpeed) < OperatorConstants.kPilotDeadband && Math.abs(rotSpeed) < OperatorConstants.kPilotDeadband){
      idleTimer.start();
    } else {
      idleTimer.stop();
      idleTimer.reset();
    }
    if (idleTimer.get() > OperatorConstants.kIdleTimeoutSeconds){
      isRobotIdle = true;
    } else {
      isRobotIdle = false;
    }


    //apply deadband
    xSpeed = Math.abs(xSpeed) > OperatorConstants.kPilotDeadband ? xSpeed : 0.0; // <
    ySpeed = Math.abs(ySpeed) > OperatorConstants.kPilotDeadband ? ySpeed : 0.0; //if requested speed is less than a certain value, set to 0 to prevent drift and ensure the movement was intentional
    rotSpeed = Math.abs(rotSpeed) > OperatorConstants.kPilotDeadband ? rotSpeed : 0.0; // <

    //apply limiter
    xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.kMaxSpeedMetersPerSecond); // sim says 8ft/s is max speed now, so remove / 2 ?
    ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.kMaxSpeedMetersPerSecond); //limit speed to max speed? idk lol
    rotSpeed = rotLimiter.calculate(rotSpeed) * DriveConstants.kMaxAngularSpeedRadiansPerSecond / 2; //same with this

    //apply field relative
    ChassisSpeeds chassisSpeeds; //construct chassis speed
    if (fieldRelativeFunction.get()){ //if field relative mode (default)
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotSpeed, swerveSubsystem.getGyroRotation2d(rotSim));
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    }

    //creates a list of swervemodule states to apply to the modules
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); //inverse kinematics that we don't have to worry about. Thanks WPIlib!

    //output module states to wheels
    swerveSubsystem.setModuleStates(moduleStates); 




    //TODO: If robot is stopped, set modules to brake mode, else set to coast mode
    //TODO: If robot is stopped, set modules to X formation, else use ChassisSpeeds module states
          //Could check if all inputs are under their deadbands, if so, start a timer to see if robot is stopped for a certain amount of time
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
