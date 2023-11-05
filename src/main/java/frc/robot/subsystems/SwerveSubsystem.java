// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveBaseConstants;

public class SwerveSubsystem extends SubsystemBase {
  //Swerve module objects from our class we created with each of the parameters we need (x4)
  private final SwerveModule frontLeftModule = new SwerveModule(
    DriveBaseConstants.kFrontLeftDriveMotorID,
    DriveBaseConstants.kFrontLeftTurnMotorID,
    DriveBaseConstants.kFrontLeftDriveEncoderReversed,
    DriveBaseConstants.kFrontLeftTurnEncoderReversed,
    DriveBaseConstants.kFrontLeftAbsoluteEncoderID,
    DriveBaseConstants.kFrontLeftAbsoluteEncoderOffsetRadians,
    DriveBaseConstants.kFrontLeftAbsoluteEncoderReversed
  );
  private final SwerveModule frontRightModule = new SwerveModule(
    DriveBaseConstants.kFrontRightDriveMotorID,
    DriveBaseConstants.kFrontRightTurnMotorID,
    DriveBaseConstants.kFrontRightDriveEncoderReversed,
    DriveBaseConstants.kFrontRightTurnEncoderReversed,
    DriveBaseConstants.kFrontRightAbsoluteEncoderID,
    DriveBaseConstants.kFrontRightAbsoluteEncoderOffsetRadians,
    DriveBaseConstants.kFrontRightAbsoluteEncoderReversed
  );
  private final SwerveModule backLeftModule = new SwerveModule(
    DriveBaseConstants.kBackLeftDriveMotorID,
    DriveBaseConstants.kBackLeftTurnMotorID,
    DriveBaseConstants.kBackLeftDriveEncoderReversed,
    DriveBaseConstants.kBackLeftTurnEncoderReversed,
    DriveBaseConstants.kBackLeftAbsoluteEncoderID,
    DriveBaseConstants.kBackLeftAbsoluteEncoderOffsetRadians,
    DriveBaseConstants.kBackLeftAbsoluteEncoderReversed
  );
  private final SwerveModule backRightModule = new SwerveModule(
    DriveBaseConstants.kBackRightDriveMotorID,
    DriveBaseConstants.kBackRightTurnMotorID,
    DriveBaseConstants.kBackRightDriveEncoderReversed,
    DriveBaseConstants.kBackRightTurnEncoderReversed,
    DriveBaseConstants.kBackRightAbsoluteEncoderID,
    DriveBaseConstants.kBackRightAbsoluteEncoderOffsetRadians,
    DriveBaseConstants.kBackRightAbsoluteEncoderReversed
  );

  private ADIS16470_IMU gyro = new ADIS16470_IMU(); //gyro on the RoboRIO

  private Field2d field = new Field2d(); //field image on the dashboard
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry( //initialize odometry for the robot (position and rotation mainly to display on the dashboard)
    DriveConstants.kDriveKinematics, //kinematics of the robot (size and location of the wheels)
    getGyroRotation2d(), //initial rotation in a Rotation2d object 
    new SwerveModulePosition[]{ //list of each of the modules' encoder positions (wheel distance driven and Rotation2d object of the wheel's angle)
      frontLeftModule.getModulePosition(),
      frontRightModule.getModulePosition(),
      backLeftModule.getModulePosition(),
      backRightModule.getModulePosition()},
      new Pose2d(0, 0, getGyroRotation2d())); //initial position of the robot in a Pose2d object (x, y, and rotation)

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> { 
      try {
        Thread.sleep(1000); // Wait for gyro to calibrate,
        zeroGyro();         // then zero it
      } catch (Exception e){
        System.out.println(e);
      }
    }).start();
    SmartDashboard.putData("Field", field); //display the field image on the dashboard
    field.setRobotPose(odometry.getPoseMeters()); //set the field image's robot position to the odometry's (initial) robot position
  }

  public void zeroGyro(){ //reset gyro
    gyro.reset();
  }
  public double getGyroAngleDegrees(){ //get gyro angle in degrees
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
  public Rotation2d getGyroRotation2d(){ //get gyro angle in a Rotation2d object
    return Rotation2d.fromDegrees(getGyroAngleDegrees());
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run (~50 times per second) 
    SmartDashboard.putNumber("Robot Heading", getGyroAngleDegrees()); //display gyro angle on the dashboard
    odometry.update( //constantly update odometry with the current module positions and gyro angle
      getGyroRotation2d(),
      new SwerveModulePosition[]{
        frontLeftModule.getModulePosition(),
        frontRightModule.getModulePosition(),
        backLeftModule.getModulePosition(),
        backRightModule.getModulePosition()
      });
    field.setRobotPose( //constantly update the robot image on the dashboard field image (x meters, y meters, and rotation2d object)
      odometry.getPoseMeters().getX(),
      odometry.getPoseMeters().getY(),
      getGyroRotation2d());
  }

  public void stopModules(){ //stop all modules
    frontLeftModule.stopMotors();
    frontRightModule.stopMotors();
    backLeftModule.stopMotors();
    backRightModule.stopMotors();
  }
  public void brakeModules(){ //brake all modules
    frontLeftModule.brakeMotors();
    frontRightModule.brakeMotors();
    backLeftModule.brakeMotors();
    backRightModule.brakeMotors();
  }
  public void coastMotors(){
    frontLeftModule.coastMotors();
    frontRightModule.coastMotors();
    backLeftModule.coastMotors();
    backRightModule.coastMotors();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){ //set the desired states of each module with a given list of module states (desiredStates)
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond); //"Renormalizes the wheel speeds if any individual speed is above the specified maximum."
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }
}