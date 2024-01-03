// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double kWheelDiameterMeters = 0; //measure
  public static final double kDriveMotorGearRatio = 0; //measure
  public static final double kTurningMotorGearRatio = 0; //measure
  public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
  public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
  public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
  public static final double kPTurning = 0;

  public static final double kPhysiycalMaxSpeedMetersPerSecond = .5;
  
  public static final double kTeleDriveMaxDriveAccelerationUnitsPerSecond = 0;
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0;
  public static final double kTeleDriveMaxAngularSpeedUnitsPerSecond = .5;
  public static final double kTeleDriveMaxSpeedMetersPerSecond = .5;
  
  
  public static final double kTrackWidth = Units.inchesToMeters(0); // Distance between right and left wheels
  public static final double kWheelBase = Units.inchesToMeters(0); //Distance between front and back wheels
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    new Translation2d(kWheelBase / 2, kTrackWidth / 2),
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    //in 2d space, gives us the coordinates of each module and then we can use this to preform commands off of it with the passed in data
  );
  
  //Controller Constants
  public static final int kDriverControllerPort = 0;
  public static final int kLeftYAxis = 0;
  public static final int kLeftXAxis = 1;
  public static final int kRightXAxis = 2;
  public static final int kButtonA = 0;
  //motor id constants
  public static final int frontLeftDrive = 0;
  public static final int frontLeftTurn = 1;
  public static final int frontRightDrive = 2;
  public static final int frontRightTurn = 3;
  public static final int backLeftDrive = 4;
  public static final int backLeftTurn = 5;
  public static final int backRightDrive = 6;
  public static final int backRightTurn = 7;

  //Encoder id constants
  public static final int frontLeftEncoder = 0;
  public static final int frontRightEncoder = 1;
  public static final int backLeftEncoder = 2;
  public static final int backRightEncoder = 3;
}
