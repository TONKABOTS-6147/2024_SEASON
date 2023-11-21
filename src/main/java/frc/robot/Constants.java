// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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




//below is from 0 to auto github
    //     public static final class ModuleConstants {
//       public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
//       public static final double kDriveMotorGearRatio = 1 / 5.8462;
//       public static final double kTurningMotorGearRatio = 1 / 18.0;
//       public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
//       public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
//       public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
//       public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
//       public static final double kPTurning = 0.5;
//   }

  public static final class DriveConstants {
      //TODO: go through all values and make sure they are correct for our robot!!!
      public static final double kTrackWidth = Units.inchesToMeters(23.00);
      // Distance between right and left wheels
      public static final double kWheelBase = Units.inchesToMeters(23.00);
      // Distance between front and back wheels
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

      public static final int kFrontLeftDriveMotorPort = 14; //A
      public static final int kBackLeftDriveMotorPort = 17; //B
      public static final int kFrontRightDriveMotorPort = 23; //D
      public static final int kBackRightDriveMotorPort = 20; //C

      public static final int kFrontLeftTurningMotorPort = 15;
      public static final int kBackLeftTurningMotorPort = 18;
      public static final int kFrontRightTurningMotorPort = 24;
      public static final int kBackRightTurningMotorPort = 21;

      public static final boolean kFrontLeftTurningEncoderReversed = true;
      public static final boolean kBackLeftTurningEncoderReversed = true;
      public static final boolean kFrontRightTurningEncoderReversed = true;
      public static final boolean kBackRightTurningEncoderReversed = true;

      public static final boolean kFrontLeftDriveEncoderReversed = true;
      public static final boolean kBackLeftDriveEncoderReversed = true;
      public static final boolean kFrontRightDriveEncoderReversed = false;
      public static final boolean kBackRightDriveEncoderReversed = false;

      public static final int kFrontLeftDriveAbsoluteEncoderPort = 13;
      public static final int kBackLeftDriveAbsoluteEncoderPort = 16;
      public static final int kFrontRightDriveAbsoluteEncoderPort = 22;
      public static final int kBackRightDriveAbsoluteEncoderPort = 19;

      public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
      public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
      public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
      public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

      // TODO: radians?
      public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 35.453125;
      public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 64.072265625;
      public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 19.775390625;
      public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 333.544921875;

      public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

      public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

      // For Rate Limiter
      public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

//   public static final class AutoConstants {
//       public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
//       public static final double kMaxAngularSpeedRadiansPerSecond = //
//               DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
//       public static final double kMaxAccelerationMetersPerSecondSquared = 3;
//       public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
//       public static final double kPXController = 1.5;
//       public static final double kPYController = 1.5;
//       public static final double kPThetaController = 3;

//       public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
//               new TrapezoidProfile.Constraints(
//                       kMaxAngularSpeedRadiansPerSecond,
//                       kMaxAngularAccelerationRadiansPerSecondSquared);
//   }

  public static final class OIConstants {
      public static final int kDriverControllerPort = 0;

      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;
      public static final int kDriverRotAxis = 4;
      public static final int kDriverFieldOrientedButtonIdx = 1;

      public static final double kDeadband = 0.05;
  }
// }
  }
}
