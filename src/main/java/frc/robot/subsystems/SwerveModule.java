

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  private final TalonFX driveMotor;

  private final TalonFX turningMotor;

  private final CANCoder absEncoder;

  // private final PIDController turningPIDController;

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
    return 1.1;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
