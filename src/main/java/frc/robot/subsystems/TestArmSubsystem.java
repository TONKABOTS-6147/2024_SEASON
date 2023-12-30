// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestArmSubsystem extends SubsystemBase {

  private ShuffleboardTab tab;
  private GenericEntry positionDegrees;
  private GenericEntry motionProfileSwitch;
  private CANSparkMax m_motor;

  public TestArmSubsystem() {
    tab = Shuffleboard.getTab("Arm");
    positionDegrees = tab.add("Position Degrees", 0.0).withWidget(BuiltInWidgets.kGyro).getEntry();
    motionProfileSwitch = tab.add("Motion Profile", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  }

  public void setPosition(double pos){
    double setpoint = degToEncoderCounts(pos);
    if (motionProfileSwitch.getBoolean(false)){
      motionProfileControl(setpoint);
    } else {
      pidControl(setpoint);
    }
    positionDegrees.setDouble(pos);
  }

  public double degToEncoderCounts(double pos){
    return pos;
  }

  public void pidControl(double setpoint){}

  public void motionProfileControl(double setpoint){}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
