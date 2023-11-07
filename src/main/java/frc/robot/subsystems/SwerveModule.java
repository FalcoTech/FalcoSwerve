// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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

  // private final AnalogInput absoluteEncoder;
  private final CANCoder absoluteEncoder;
  private CANCoderConfiguration AEconfig;

  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRadians;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    this.absoluteEncoderOffsetRadians = absoluteEncoderOffset; //Encoder offset in radians
    this.absoluteEncoderReversed = absoluteEncoderReversed; // Encoder reversed or not
    
    absoluteEncoder = new CANCoder(absoluteEncoderID); // Absolute encoder object
    AEconfig = new CANCoderConfiguration();

    AEconfig.sensorCoefficient = 2 * Math.PI / 4096.0; //value multiplied to get radians (divide by 4096 encoder ticks for % of full rotation, multiple 2pi to convert to radians)
    AEconfig.unitString = "rad"; //set absolute encoder unit to radians
    AEconfig.sensorDirection = absoluteEncoderReversed; //set absolute encoder to be reversed or not
    AEconfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180; //set absolute encoder to return values from -180 to 180 degrees

    AEconfig.sensorTimeBase = SensorTimeBase.PerSecond; //set absolute encoder to return radians per second instead of rotations/ticks per something (idk what default is lol)
    
    absoluteEncoder.configAllSettings(AEconfig); //apply settings to absolute encoder

    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless); // Drive motor object
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless); // Turn motor object

    driveMotor.setInverted(driveMotorReversed); // Set drive motor to reversed or not
    turnMotor.setInverted(turnMotorReversed); // Set turn motor to reversed or not

    driveEncoder = driveMotor.getEncoder(); // Drive Relative Encoder
    turnEncoder = turnMotor.getEncoder(); // Turn Relative Encoder

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter); // Set drive encoder distance unit to meters instead of rotations/ticks
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec); //set drive encoder velocity unit to meters per second instead of RPM
    turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Radian); // Set turn encoder distance unit to radians instead of rotations/ticks
    turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadianPerSec); // Set turn encoder velocity unit to radians per second instead of RPM

    turnPIDController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning); // Create PID controller for turning
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
    double angle = absoluteEncoder.getPosition(); // Get absolute encoder angle
    return angle - absoluteEncoderOffsetRadians; // Subtract offset for "actual" angle
  }
  public Rotation2d getAbsoluteEncoderRotation2d(){ // Absolute encoder Rotation2d object
    new Rotation2d();
    return Rotation2d.fromRadians(getAbsoluteEncoderRadians());
  }

  public void resetEncoders(){ //Reset drive encoder and set turn encoder to absolute encoder
    driveEncoder.setPosition(0.0);
    turnEncoder.setPosition(getAbsoluteEncoderRadians());
  }

  public SwerveModuleState getModuleState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRadians()));
  }
  public SwerveModulePosition getModulePosition(){
    return new SwerveModulePosition(getDrivePosition(), getAbsoluteEncoderRotation2d()); //getAbsoluteEncoderRotation2d() returns a Rotation2d object
  }

  public void stopMotors(){
    driveMotor.set(0);
    turnMotor.set(0);
  }
  public void coastMotors(){
    driveMotor.setIdleMode(IdleMode.kCoast);
    turnMotor.setIdleMode(IdleMode.kCoast);
  }
  public void brakeMotors(){
    driveMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setDesiredState(SwerveModuleState state){ // Set desired state of the module takes a module state object calculated in the command with the given driver parameters (speed, angle)
    if (Math.abs(state.speedMetersPerSecond) < 0.001){ // If speed is less than 0.001 m/s, stop motors, and no need to apply power to motors, so return (stop and get out of the function)
      stopMotors();
      return;
    }
    state = SwerveModuleState.optimize(state, getModuleState().angle); // Optimize the state to get the shortest path to the desired angle

    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond); // Set drive motor to the desired speed
    
    turnMotor.set(turnPIDController.calculate(getTurnPosition(), state.angle.getRadians())); // Set turn motor to the desired angle
    
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString()); // print debug info to smartdashboard
  }
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  public void setModuleAngle(double degrees){
    turnMotor.set(turnPIDController.calculate(getTurnPosition(), Math.toRadians(degrees)));
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getDeviceID() + "] AbsEnc Rad", getAbsoluteEncoderRadians()); // print debug info to smartdashboard
    SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getDeviceID() + "] AbsEnc Deg", Math.toDegrees(getAbsoluteEncoderRadians())); // print debug info to smartdashboard

    
  }
}
