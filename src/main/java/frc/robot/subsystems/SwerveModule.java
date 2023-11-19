

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  private final TalonFX driveMotor;
  private final TalonFX turningMotor;
  private final CANCoder absEncoder;

  private final double absEncoderOffsetRad; // angle; prob the same as angleOffset in 364's code
  private final boolean absEncoderReversed;


  public SwerveModule(int driveMotorID, int turningMotorID, 
                      boolean driveMotorReversed, boolean turningMotorReversed, 
                      int absEncoderID, double absEncoderOffset, 
                      boolean absEncoderReversed) {  
    
    this.absEncoderOffsetRad = absEncoderOffset; // degrees or radians!??
    this.absEncoderReversed = absEncoderReversed; // needed?

    this.absEncoder = new CANCoder(absEncoderID);

    this.driveMotor = new TalonFX(driveMotorID);
    this.turningMotor = new TalonFX(turningMotorID);
    driveMotor.configFactoryDefault();
    turningMotor.configFactoryDefault();

    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setNeutralMode(NeutralMode.Brake);

    // ticks may need to be converted to meters/second and possibly radians

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveMotor.config_kP(0, 0.1);
    turningMotor.config_kP(0, 0.1);
  }

  public void resetPositon (TalonFX motor) {
    motor.setSelectedSensorPosition(0);
  }

  // Get Positions:
  public double getDrivePosition() {
    return this.driveMotor.getSelectedSensorPosition(); // raw units
  }

  public double getTurningPosition() {
    return this.turningMotor.getSelectedSensorPosition(); // raw units
  }

  // Get Velocities:
  public double getDriveVelocity() {
    return this.driveMotor.getSelectedSensorVelocity(); // raw units / 100ms (milliseconds)
  }

  public double getTurningVelocity() {
    return this.turningMotor.getSelectedSensorVelocity(); // raw units / 100ms (milliseconds)
  }

  // Encoder Angle Stuff:
  public double getAbsEncoderRad() {
    return 0; //TODO: make this method return the offset of the wheels found by the abs encoder
  }

  public void initEncoderPosition(){
    this.driveMotor.setSelectedSensorPosition(0);
    double turningPos = getAbsEncoderRad(); //TODO: same as above todo, convert init rads to falcon encoder units and pass in below to account for offset
    this.turningMotor.setSelectedSensorPosition(turningPos);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    //TODO: look into what units the swervemodulestate takes, i don't think it takes what we are giving it currently
  }
  public void setDesiredState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle); //TODO: not sure if this will work or not... should make it never have to turn more than 90 degrees.
    driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond);
    turningMotor.setSelectedSensorPosition(state.angle.getRadians()); //TODO: this doesn't take radians, convert radians to encoder units, make sure it allows for 
    SmartDashboard.putString("Swerve state", state.toString()); //TODO: currently there will be four modules all with the same state names...

  }

  public void stop(){
    driveMotor.set(ControlMode.PercentOutput, 0);
    turningMotor.set(ControlMode.PercentOutput, 0);
    //TODO: not sure if this is needed because we are using closed loop velocity control rather than percent output.
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
