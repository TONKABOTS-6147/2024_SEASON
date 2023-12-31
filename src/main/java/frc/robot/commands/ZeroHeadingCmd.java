// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroHeadingCmd extends CommandBase {
  /** Creates a new ZeroHeadingCmd. */
  public SwerveSubsystem m_swerveSubsystem; 
  
  public ZeroHeadingCmd(SwerveSubsystem swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveSubsystem.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Nothing
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Nothing
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
