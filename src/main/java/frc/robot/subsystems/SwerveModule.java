

package frc.robot.subsystems;


import javax.print.attribute.standard.PrinterInfo;
import javax.security.auth.x500.X500Principal;
import javax.swing.plaf.basic.BasicBorders.RadioButtonBorder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;

public class SwerveModule extends SubsystemBase {

  private final TalonFX driveMotor;
  private final TalonFX turningMotor;
  private final CANCoder absEncoder;
  private final char id;
  private final String position;
  private final double absEncoderOffset;
  // private final boolean absEncoderReversed;


  public SwerveModule(int driveMotorID, int turningMotorID, 
                      boolean driveMotorReversed, boolean turningMotorReversed, 
                      int absEncoderID, double absEncoderOffset, 
                      boolean absEncoderReversed, char id, String position) {  
     
    this.absEncoderOffset = absEncoderOffset;
    // this.absEncoderReversed = absEncoderReversed; // needed?

    this.absEncoder = new CANCoder(absEncoderID);

    this.driveMotor = new TalonFX(driveMotorID);
    this.turningMotor = new TalonFX(turningMotorID);
    this.id = id;
    this.position = position;
    driveMotor.configFactoryDefault();
    turningMotor.configFactoryDefault();
    absEncoder.configFactoryDefault();

    absEncoder.configSensorDirection(absEncoderReversed);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setNeutralMode(NeutralMode.Brake);

    // ticks may need to be converted to meters/second and possibly radians

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);
    

    driveMotor.config_kP(0, 0.05);
    turningMotor.config_kP(0, 0.1);
  }

  public void resetPositon (TalonFX motor) {
    motor.setSelectedSensorPosition(0);
  }

  // Get Positions:
  public double getDrivePosition() {
    return this.driveMotor.getSelectedSensorPosition(); // raw units
  }

  public double getTurningPositionRad() {
    double turnDegrees = (this.turningMotor.getSelectedSensorPosition() * 360.0) / 2048.0 * ChassisConstants.angleGearRatio; 
    return Math.toRadians(turnDegrees);
  }
  public double getTurningPositionDeg() {
    double turnDegrees = (this.turningMotor.getSelectedSensorPosition() * 360.0) / 2048.0 * ChassisConstants.angleGearRatio; 
    return turnDegrees;
  }

  // Get Velocities:
  public double getDriveVelocity() {
    // starts in raw units / 100ms (milliseconds)
    //m / second
    return (this.driveMotor.getSelectedSensorVelocity()*10) * (ChassisConstants.wheelCircumference / 2048);
  }

  public double getTurningVelocity() {
    double ticksPer = this.turningMotor.getSelectedSensorVelocity(); // raw units / 100ms (milliseconds)
    ticksPer *= 10; // raw units / second (1000ms)
    return ticksPer * (360 / 2048);
  }

  public double convertOffset() {
    double adjustedAbsEncoderPos = absEncoder.getAbsolutePosition() - this.absEncoderOffset;
    System.out.println(adjustedAbsEncoderPos);
    double ticks = adjustedAbsEncoderPos * (2048.0 / 360.0); // converts degrees to falcon encoder units. 
    return ticks * ChassisConstants.angleGearRatio;
  }

  public void initEncoderPosition(){
    double turningPos = convertOffset();
    this.turningMotor.setSelectedSensorPosition(turningPos);
    System.out.println(turningPos);
  }

  //used for optimize later...
  // public SwerveModuleState getState() {
  //   return new SwerveModuleState(getDriveVelocity() /* TO METERS PER SECOND */, new Rotation2d(getTurningPosition()));
  // }
  

  public void setDesiredState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    // state = SwerveModuleState.optimize(state, getState().angle); 

    // Convert state speed and position to CTRE friendly ones
    double speedMPS = state.speedMetersPerSecond;
    double speedTicksPer100ms = (speedMPS  / 10) * (2048 / ChassisConstants.wheelCircumference);
    double speedAdjustedForRatio = speedTicksPer100ms * ChassisConstants.driveGearRatio; 
  
    // Converts -180 to 0 to 180 to 360
    double targetAngle = state.angle.getDegrees();
    double currentPosition = getTurningPositionDeg();
    if (currentPosition < 0){
      currentPosition += 360;
    }
    if (targetAngle < 0){
      targetAngle += 360;
    }
    
    if ((Math.abs(targetAngle - currentPosition) > 90) && (Math.abs(targetAngle - currentPosition) < 270)) {
      targetAngle = (targetAngle + 180.0) % 360.0;
      SmartDashboard.putString("current deg" + this.id + " | " + this.position + " Module" + ": ", Double.toString(getTurningPositionDeg())); 
      SmartDashboard.putString("target deg" + this.id + " | " + this.position + " Module" + ": ", Double.toString(targetAngle)); //NOTE: what is the output really?! we will see on dashboard when startup

      speedAdjustedForRatio *= -1;
    }
    double angleToEncoderUnits = targetAngle * (2048.0 / 360.0);
    double angleAdjustedForRatio = angleToEncoderUnits * ChassisConstants.angleGearRatio;


    // state = SwerveModuleState.optimize(state, new Rotation2d(getTurningPosition()));


    // Convert state speed and position to CTRE friendly ones
    // double speedMPS = state.speedMetersPerSecond;
    // double speedTicksPer100ms = (speedMPS  / 10) * (2048 / ChassisConstants.wheelCircumference);
    // double speedAdjustedForRatio = speedTicksPer100ms * ChassisConstants.driveGearRatio; 
  
    // double targetAngle = state.angle.getDegrees();  
    // double angleToEncoderUnits = targetAngle * (2048.0 / 360.0);
    // double angleAdjustedForRatio = angleToEncoderUnits * ChassisConstants.angleGearRatio;

    SmartDashboard.putString("Swerve state " + this.id + " | " + this.position + " Module" + ": ", state.toString()); //NOTE: what is the output really?! we will see on dashboard when startup

    driveMotor.set(ControlMode.Velocity, speedAdjustedForRatio); // ticks / 100ms
    turningMotor.set(ControlMode.Position, angleAdjustedForRatio);
  }

  public void stop() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    turningMotor.set(ControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Absolute Encoder Position" + this.id + ": ", absEncoder.getAbsolutePosition()); // degs
    SmartDashboard.putNumber("CONV " + this.id + ": ", (this.turningMotor.getSelectedSensorPosition() * 360) / 2048); // degs
    SmartDashboard.putNumber("RAW " + this.id + ": ", this.turningMotor.getSelectedSensorPosition()); // degs
  }
}
