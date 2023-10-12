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

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeftModule = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorID,
    DriveConstants.kFrontLeftTurnMotorID,
    DriveConstants.kFrontLeftDriveEncoderReversed,
    DriveConstants.kFrontLeftTurnEncoderReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderID,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRadians,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
  );

  private final SwerveModule frontRightModule = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorID,
    DriveConstants.kFrontRightTurnMotorID,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurnEncoderReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderID,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRadians,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
  );

  private final SwerveModule backLeftModule = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorID,
    DriveConstants.kBackLeftTurnMotorID,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurnEncoderReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderID,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRadians,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
  );

  private final SwerveModule backRightModule = new SwerveModule(
    DriveConstants.kBackRightDriveMotorID,
    DriveConstants.kBackRightTurnMotorID,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurnEncoderReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderID,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRadians,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed
  );

  private ADIS16470_IMU gyro = new ADIS16470_IMU();

  private Field2d field = new Field2d();

  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    getRotation2d(),
    new SwerveModulePosition[] {
      new SwerveModulePosition(frontLeftModule.getDrivePosition(), frontLeftModule.getAbsoluteEncoderRotation2d()),
      new SwerveModulePosition(frontRightModule.getDrivePosition(), frontRightModule.getAbsoluteEncoderRotation2d()),
      new SwerveModulePosition(backLeftModule.getDrivePosition(), backLeftModule.getAbsoluteEncoderRotation2d()),
      new SwerveModulePosition(backRightModule.getDrivePosition(), backRightModule.getAbsoluteEncoderRotation2d())
    }, new Pose2d(0, 0, new Rotation2d())
  );

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> { // Wait for gyro to calibrate, then zero it
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){
        System.out.println(e);
      }
    }).start();

    field.setRobotPose(0, 0, new Rotation2d());
    SmartDashboard.putData(field);
  }

  public void zeroHeading(){
    gyro.reset();
  }
  public double getGyroHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getGyroHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var gyroAngle = getRotation2d();
    SmartDashboard.putNumber("Robot Heading", getGyroHeading());

    odometry.update(gyroAngle, 
      new SwerveModulePosition[] {
        new SwerveModulePosition(frontLeftModule.getDrivePosition(), frontLeftModule.getAbsoluteEncoderRotation2d()),
        new SwerveModulePosition(frontRightModule.getDrivePosition(), frontRightModule.getAbsoluteEncoderRotation2d()),
        new SwerveModulePosition(backLeftModule.getDrivePosition(), backLeftModule.getAbsoluteEncoderRotation2d()),
        new SwerveModulePosition(backRightModule.getDrivePosition(), backRightModule.getAbsoluteEncoderRotation2d())
      }
    );

  }

  public void stopModules(){
    frontLeftModule.stop();
    frontRightModule.stop();
    backLeftModule.stop();
    backRightModule.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

}