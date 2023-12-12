

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

    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setNeutralMode(NeutralMode.Brake);

    // ticks may need to be converted to meters/second and possibly radians

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveMotor.config_kP(0, 0.05);
    turningMotor.config_kP(0, 0.6);
  }

  public void resetPositon (TalonFX motor) {
    motor.setSelectedSensorPosition(0);
  }

  // Get Positions:
  public double getDrivePosition() {
    return this.driveMotor.getSelectedSensorPosition(); // raw units
  }

  public double getTurningPosition() {
    double turnDegrees = (this.turningMotor.getSelectedSensorPosition() * 360) / 2048; // wants radians
    return Math.toRadians(turnDegrees);
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

  // Encoder Angle Stuff:
  public double getAbsEncoderInFalconUnits() {
    double adjustedAbsEncoderPos = absEncoder.getAbsolutePosition() - this.absEncoderOffset;  //double checked
    return (adjustedAbsEncoderPos * 2048) / 360; // converts degrees to falcon encoder units.  //double checked
  }

  public void initEncoderPosition(){
    double turningPos = getAbsEncoderInFalconUnits();
    this.turningMotor.setSelectedSensorPosition(turningPos);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity() /* TO METERS PER SECOND */, new Rotation2d(getTurningPosition()));
  }
  

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
  
    double angleDeg = state.angle.getDegrees();
    double angleTicksPer100ms = angleDeg * (2048 / 360);
    double angleAdjustedForRatio = angleTicksPer100ms * ChassisConstants.angleGearRatio;


    SmartDashboard.putString("Swerve state " + this.id + " | " + this.position + " Module" + ": ", state.toString()); //NOTE: what is the output really?! we will see on dashboard when startup


    driveMotor.set(ControlMode.Velocity, speedAdjustedForRatio); // ticks / 100ms
    turningMotor.set(ControlMode.Position, angleAdjustedForRatio);
  }

  public void stop() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    turningMotor.set(ControlMode.PercentOutput, 0);
    //TODO: not sure if this is needed because we are using closed loop velocity control rather than percent output.
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Absolute Encoder Position" + this.id + ": ", absEncoder.getAbsolutePosition()); // degs
  }
}
