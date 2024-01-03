// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestArmSubsystem extends SubsystemBase {

  private final double gearRatio = (20.0/1.0);
  private final int canID = 50;
  private final int pidID = 0;
  private final int profileID = 1;
  private ShuffleboardTab tab;
  // private GenericEntry posDegCurrent;
  private GenericEntry posDegWanted;
  private GenericEntry motionProfileSwitch;
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kPpid, kIpid, kDpid, kIzpid, kFFpid, kMaxOutputpid, kMinOutputpid, maxRPMpid;
  public double kPProfile, kIProfile, kDProfile, kIzProfile, kFFProfile, kMaxOutputProfile, kMinOutputProfile, maxRPMProfile, maxVelProfile, minVelProfile, maxAccProfile, allowedErrProfile;

  public TestArmSubsystem() {
    tab = Shuffleboard.getTab("Arm");
    // posDegCurrent = tab.add("Current Degrees", 0.0).withWidget(BuiltInWidgets.kGyro).getEntry();
    posDegWanted = tab.add("Wanted Degrees", 0.0).getEntry();
    motionProfileSwitch = tab.add("Motion Profile", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    m_motor = new CANSparkMax(canID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    //PID setup slot setup
    kPpid = 0.1;
    // kIpid = 0;
    // kDpid = 0;
    // kIzpid = 0;
    // kFFpid = 0;
    kMinOutputpid = -0.2;
    kMaxOutputpid = 0.2;
    m_pidController.setP(kPpid, pidID);
    // m_pidController.setI(kIpid, pidID);
    // m_pidController.setD(kDpid, pidID);
    // m_pidController.setIZone(kIzpid, pidID);
    // m_pidController.setFF(kFFpid, pidID);
    m_pidController.setOutputRange(kMinOutputpid, kMaxOutputpid, pidID);

    //Smart Motion slot setup
    kPProfile = 0.05;
    kIProfile = 0;
    kDProfile = 0;
    kIzProfile = 0;
    kFFProfile = 0;
    kMinOutputProfile = -0.5;
    kMaxOutputProfile = 0.5;
    maxVelProfile = 30; // rpm
    maxAccProfile = 20; //rpm
    m_pidController.setP(kPProfile, profileID);
    m_pidController.setI(kIProfile, profileID);
    m_pidController.setD(kDProfile, profileID);
    m_pidController.setIZone(kIzProfile, profileID);
    m_pidController.setFF(kFFProfile, profileID);
    m_pidController.setOutputRange(kMinOutputProfile, kMaxOutputProfile, profileID);
    m_pidController.setSmartMotionMaxVelocity(maxVelProfile, profileID);
    m_pidController.setSmartMotionMinOutputVelocity(minVelProfile, profileID);
    m_pidController.setSmartMotionMaxAccel(maxAccProfile, profileID);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErrProfile, profileID);
  }

  public void setPosition(){
    double pos = posDegWanted.getDouble(0.0);
    double setpoint = degToEncoderCounts(pos);
    if (motionProfileSwitch.getBoolean(false)){
      m_pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion, profileID);
    } else {
      m_pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition, pidID);
    }
  }

  public double degToEncoderCounts(double pos){
    double setpoint = ((pos/360.0) * gearRatio);
    return setpoint;
  }

  public double encoderCountsToDeg(double pos){
    double deg = (360.0*pos) / gearRatio;
    return deg;
  }

  @Override
  public void periodic() {
    // posDegCurrent.setDouble(encoderCountsToDeg(m_encoder.getPosition()));
    SmartDashboard.putNumber("Current Degrees", encoderCountsToDeg(m_encoder.getPosition()));
    SmartDashboard.putNumber("Current Encoder Pos", m_encoder.getPosition());
  }
}
