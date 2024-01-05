// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

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
   
  public static final ADIS16470_IMU imu = new ADIS16470_IMU();
  public SwerveModulePosition[] modulePositions = null;
  // private final ADIS16470_IMU imu = new ADIS16470_IMU();

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), null);
 
  public SwerveSubsystem() {

    new Thread(() -> {
      // This thread will allow other processes to happen WHILE the gyro resets... not necessary probably *BUT*  JIC!
      try {
        Thread.sleep(1000 /* ms */ );
        zeroHeading();
      } catch (Exception e) { 

      }
    }).start();
  
    frontLeft.initEncoderPosition();
    frontRight.initEncoderPosition();
    backLeft.initEncoderPosition();
    backRight.initEncoderPosition();

    // AutoBuilder.configureHolonomic(
    //     this::getPose, // Robot pose supplier
    //     this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //     this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //     this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //         new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //         new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //         4.5, // Max module speed, in m/s
    //         0.4, // Drive base radius in meters. Distance from robot center to furthest module.
    //         new ReplanningConfig() // Default path replanning config. See the API for the options here
    //     ),
    //     this // Reference to this subsystem to set requirements
    // );
  }


  public void zeroHeading() { 
    imu.reset();
  }

  public double getHeading(){
    return imu.getAngle() *-1;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SwerveModulePosition frontLeftPosition = new SwerveModulePosition(frontLeft.getDrivePosition(), Rotation2d.fromDegrees(frontLeft.getTurningPositionDeg()));
    SwerveModulePosition frontRightPosition = new SwerveModulePosition(frontRight.getDrivePosition(), Rotation2d.fromDegrees(frontRight.getTurningPositionDeg()));
    SwerveModulePosition backLeftPosition = new SwerveModulePosition(backLeft.getDrivePosition(), Rotation2d.fromDegrees(backLeft.getTurningPositionDeg()));
    SwerveModulePosition backRightPosition = new SwerveModulePosition(backRight.getDrivePosition(), Rotation2d.fromDegrees(backRight.getTurningPositionDeg()));

    this.modulePositions = new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition};
    odometer.update(getRotation2d(), modulePositions);
    SmartDashboard.putNumber("Robot Heading: ", getHeading());
    SmartDashboard.putString("Robot Location: ", getPose().getTranslation().toString());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), this.modulePositions, pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[]{
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
    return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
  }

  public void driveRobotRelative() {
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(getRobotRelativeSpeeds()));
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
