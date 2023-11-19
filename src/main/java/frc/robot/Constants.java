// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ChassisConstants {
    /* Chassis Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73);
    public static final double wheelBase = Units.inchesToMeters(21.73);
    public static final double swerveWheelDiameterMeters = Units.inchesToMeters(4.0);


    /* Module Gear Ratios */
    // TODO: do this
    // public static final double driveGearRatio = chosenModule.driveGearRatio;
    // public static final double steerGearRatio = chosenModule.angleGearRatio;
  }
}
