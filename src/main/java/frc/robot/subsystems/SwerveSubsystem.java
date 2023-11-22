// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

  // TODO: MAKE SURE THAT THE IDs MATCH ON THE ACTUAL DRIVETRAIN!!!!!!!!!!!!
  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorPort,
    DriveConstants.kFrontLeftTurningMotorPort,
    DriveConstants.kFrontLeftDriveEncoderReversed,
    DriveConstants.kFrontLeftTurningEncoderReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
    'A',
    "Front Left");

  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorPort,
    DriveConstants.kFrontRightTurningMotorPort,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
    'D',
    "Front Right");

  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorPort,
    DriveConstants.kBackLeftTurningMotorPort,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurningEncoderReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
    'B',
    "Back Left");

  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorPort,
    DriveConstants.kBackRightTurningMotorPort,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurningEncoderReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
    'C',
    "Back Right");
   
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  
  public SwerveSubsystem() {

    new Thread(() -> {
      // This thread will allow other processes to happen WHILE the gyro resets... not necessary probably *BUT*  JIC!
      try {
        Thread.sleep(1000 /* ms */ );
        zeroHeading();
      } catch (Exception e) { 

      }
    }).start();
  }

  public void zeroHeading() { 
    gyro.reset(); // makes the yaw zero
  }

  public double getHeading(){
    return gyro.getYaw(); // -180 to 180 
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading: ", getHeading());
  }

  // Necessary to switch from Percent to Closed Loop Velocity...?
  public void stopModules () {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    //TODO: i think we still have to use the below line but double check and also find out why its broken im too tired to figure it out rn
    /* 
    90% sure we don't need the below line because we are using closed loop velocity control, which means that if
    we set joystick forward = x m/s, then it won't go above x; every value from -1 to 1 will be proportional to x
    */  
    
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // is .normalizeWheelSpeeds() in 0 to Auto... https://www.chiefdelphi.com/t/normalizewheelspeeds/411155
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);

  }
}
