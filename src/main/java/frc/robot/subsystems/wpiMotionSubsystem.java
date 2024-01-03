// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class wpiMotionSubsystem extends TrapezoidProfileSubsystem {

  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  double kSVolts = 0;  //static gain // ?
  double kGVolts = 0;  //gravity gain // 0.1
  double kVVoltSecondPerRad = 0; //velocity gain // 0.39
  double kAVoltSecondSquaredPerRad = 0; //acceleration gain // 0
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          kSVolts, kGVolts,
          kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);

  public wpiMotionSubsystem() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(500, 25),
        // The initial position of the mechanism
        0);
    m_motor = new CANSparkMax(50, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();
    m_pidController.setP(0.1);
    m_pidController.setOutputRange(-0.4, 0.4);

  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    double feedforward = m_feedforward.calculate(state.position, state.velocity);
    // feedforward = feedforward/12.0;
    m_pidController.setFF(feedforward);
    m_pidController.setReference(state.position, CANSparkMax.ControlType.kPosition);

  }

  public Command setArmGoalCommand(double kArmOffsetRads){
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }
}
