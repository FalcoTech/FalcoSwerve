// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  private final PIDController turnPIDController;

  private final AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRadians;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    this.absoluteEncoderOffsetRadians = absoluteEncoderOffset; //Encoder offset in radians
    this.absoluteEncoderReversed = absoluteEncoderReversed; // Encoder reversed or not
    absoluteEncoder = new AnalogInput(absoluteEncoderID); // Absolute encoder object 

    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless); // Drive motor object
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless); // Turn motor object

    driveMotor.setInverted(driveMotorReversed); // Set drive motor to reversed or not
    turnMotor.setInverted(turnMotorReversed); // Set turn motor to reversed or not

    driveEncoder = driveMotor.getEncoder(); // Drive CANCoder (Relative Encoder)
    turnEncoder = turnMotor.getEncoder(); // Turn CANCoder (Relative Encoder)

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter); // Set drive encoder distance unit to meters instead of rotations/ticks
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec); //set drive encoder velocity unit to meters per second instead of RPM
    turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Radian); // Set turn encoder distance unit to radians instead of rotations/ticks
    turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadianPerSec); // Set turn encoder velocity unit to radians per second instead of RPM

    turnPIDController = new PIDController(ModuleConstants.kPTurning, 0.0, 0.0); // Create PID controller for turning
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI); // Set PID controller to be continuous

    resetEncoders(); // Reset encoders on startup
  }

  public double getDrivePosition(){ // Relative Drive Encoder "Distance" (Meters)
    return driveEncoder.getPosition(); 
  }
  public double getTurnPosition(){ //Relative Turn Encoder "Rotation" (Radians)
    return turnEncoder.getPosition();
  }

  public double getDriveVelocity(){ //Relative Drive Encoder "Velocity" (Meters per second)
    return driveEncoder.getVelocity();
  }
  public double getTurnVelocity(){ // Relative Turn Encoder "Velocity" (Radians per second)
    return turnEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRadians(){
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V(); // Voltage read / Voltage supplied (5V) = % of full rotation
    angle *= 2.0 * Math.PI; // Convert to radians
    angle -= absoluteEncoderOffsetRadians; // Subtract offset for "actual" angle
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0); // multiply by -1 if reversed
    //            (expression) ? (value if true) : (value if false)
  }
  public Rotation2d getAbsoluteEncoderRotation2d(){
    new Rotation2d();
    return Rotation2d.fromRadians(getAbsoluteEncoderRadians());
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0.0);
    turnEncoder.setPosition(getAbsoluteEncoderRadians());
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  public void stop(){
    driveMotor.set(0);
    turnMotor.set(0);
  }
  public void setDesiredState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
    turnMotor.set(turnPIDController.calculate(getTurnPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString()); // debug info
    SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] turn angle", state.angle.getDegrees());
    SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] drive motor output", state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] turn motor output", turnPIDController.calculate(getTurnPosition(), state.angle.getRadians()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
