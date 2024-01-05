

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
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CTREModuleState;
import frc.robot.Constants.ChassisConstants;

public class SwerveModule extends SubsystemBase {

  private final TalonFX driveMotor;
  private final TalonFX turningMotor;
  private final CANCoder absEncoder;
  private final char id;
  private final String position;
  private final double absEncoderOffset;

  public SwerveModule(int driveMotorID, int turningMotorID, 
                      boolean driveMotorReversed, boolean turningMotorReversed, 
                      int absEncoderID, double absEncoderOffset, 
                      boolean absEncoderReversed, char id, String position) {  
     
    this.absEncoderOffset = absEncoderOffset;

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
    double encoderTicks = this.driveMotor.getSelectedSensorPosition();
    double distance = (encoderTicks * ChassisConstants.wheelCircumference) / (2048.0 * ChassisConstants.driveGearRatio);
    return distance;
  }

  public double getTurningPositionRad() {
    double turnDegrees = (this.turningMotor.getSelectedSensorPosition() * 360.0) / (2048.0 * ChassisConstants.angleGearRatio); 
    return Math.toRadians(turnDegrees);
  }
  public double getTurningPositionDeg() {
    double turnDegrees = (this.turningMotor.getSelectedSensorPosition() * 360.0) / (2048.0 * ChassisConstants.angleGearRatio); 
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
    double ticks = adjustedAbsEncoderPos * (2048.0 / 360.0); // converts degrees to falcon encoder units. 
    return ticks * ChassisConstants.angleGearRatio;
  }

  public void initEncoderPosition(){
    double turningPos = convertOffset();
    this.turningMotor.setSelectedSensorPosition(turningPos);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity() /* TO METERS PER SECOND */, new Rotation2d(getTurningPositionRad()));
  }
  

  public void setDesiredState(SwerveModuleState desiredState){
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    double currentAngle = getTurningPositionDeg();
    // custom optimize function from team 364 because wpi optimize function assumes continuous and falcon is not
    desiredState = CTREModuleState.optimize(desiredState, Rotation2d.fromDegrees(currentAngle)); 

    // Convert state speed and position to CTRE friendly ones
    double speedMPS = desiredState.speedMetersPerSecond;
    double speedTicksPer100ms = (speedMPS  / 10) * (2048 / ChassisConstants.wheelCircumference);
    double speedAdjustedForRatio = speedTicksPer100ms * ChassisConstants.driveGearRatio; 

    double targetAngle = desiredState.angle.getDegrees();
    double angleToEncoderUnits = targetAngle * (2048.0 / 360.0);
    double angleAdjustedForRatio = angleToEncoderUnits * ChassisConstants.angleGearRatio;

    SmartDashboard.putString("Swerve state " + this.id + " | " + this.position + " Module" + ": ", desiredState.toString()); //NOTE: what is the output really?! we will see on dashboard when startup
    driveMotor.set(ControlMode.Velocity, speedAdjustedForRatio); // ticks / 100ms
    turningMotor.set(ControlMode.Position, angleAdjustedForRatio);
  }

  public void stop() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    turningMotor.set(ControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Absolute Encoder Position" + this.id + ": ", absEncoder.getAbsolutePosition()); // degs
    SmartDashboard.putNumber("CONV " + this.id + ": ", (this.turningMotor.getSelectedSensorPosition() * 360) / 2048); // degs
    SmartDashboard.putNumber("RAW " + this.id + ": ", this.turningMotor.getSelectedSensorPosition()); // degs
  }
  
}
