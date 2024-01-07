// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import team4400.Util.Swerve.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 //TODO: PLACEHOLDER VALUES ONLY, CHANGE IDs AND PID VALUES ACCORDING TO YOUR ROBOT

 /*************** DRIVE ****************/

 /*
 * Two options: 5.50 or 6.55
 * All of this data available in
 * https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options
 */

public final class Constants {
  public static boolean needToLog = true;

  public static final class ModuleConstants{
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumerence = kWheelDiameterMeters * Math.PI;
    public static final double kDriveMotorGearRatio = 5.50; //Drive Gear Ratio, 5.50 or 6.55
    public static final double kTurningMotorGearRatio = 10.29; //Turning Gear Ratio
    public static final double kREVDriveEncoderRot2Meter = 
                                kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kREVDriveEncoderRPM2MeterPerSec = kREVDriveEncoderRot2Meter / 60;
    public static final double kCTREDriveEncoderRot2Meter = 
                                kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kCTREDriveEncoderRPM2MeterPerSec = kCTREDriveEncoderRot2Meter / 60;
    public static final double kP = 0.0,
                               kI = 0,
                               kD = 0,
                               kFF = 0.0,
                               kS = 0.0,
                               kV = 0.0,
                               kA = 0.0;
    public static final double kPTurning = 100.0;
  }

  public static final class DriveConstants{
    /* Specific module constants from FRC 95:
     * https://github.com/first95/FRC2023/blob/f0e881c39ade544b3b71936995f7f075105f0b93/Clarke/src/main/java/frc/robot/Constants.java#LL136C16-L136C23
     * Gives us a tool for a cleaner and readable swerve code
    */

    /*    
     *                   F             
     *   ┌───────┬─────────────────┬───────┐
     *   │       │                 │       │
     *   │ Mod 0 │                 │ Mod 1 │
     *   │       │                 │       │
     *   ├───────┘                 └───────┤
     *   │                                 │
     *   │            Modules              │
     * L │            Diagram              │ R
     *   │                                 │
     *   │                                 │
     *   │                                 │
     *   ├───────┐                 ┌───────┤
     *   │       │                 │       │
     *   │ Mod 3 │                 │ Mod 2 │
     *   │       │                 │       │
     *   └───────┴─────────────────┴───────┘
     *                   B
     */

     //Offsets are different in each robot and encoder;
    public static final class Module0{
      public static final int DRIVE_ID = 1;
      public static final int TURN_ID = 2;
      public static final InvertedValue driveReversed = InvertedValue.Clockwise_Positive;
      public static final InvertedValue turnReversed = InvertedValue.Clockwise_Positive;
      public static final int ABSOLUTE_ID = 0; 
      public static double encoderOffset = 0.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final class Module1{
      public static final int DRIVE_ID = 3;
      public static final int TURN_ID = 4;
      public static final InvertedValue driveReversed = InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue turnReversed = InvertedValue.Clockwise_Positive;
      public static final int ABSOLUTE_ID = 1;
      public static double encoderOffset = 0.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final class Module2{
      public static final int DRIVE_ID = 5;
      public static final int TURN_ID = 6;
      public static final InvertedValue driveReversed = InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue turnReversed = InvertedValue.Clockwise_Positive;
      public static final int ABSOLUTE_ID = 2;
      public static double encoderOffset = 0.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final class Module3{
      public static final int DRIVE_ID = 7;
      public static final int TURN_ID = 8;
      public static final InvertedValue driveReversed = InvertedValue.Clockwise_Positive;
      public static final InvertedValue turnReversed = InvertedValue.Clockwise_Positive;
      public static final int ABSOLUTE_ID = 3;
      public static double encoderOffset = 0.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final int IMU_ID = 9;

    //Distance between left and right wheels
    public static final double kTrackWidth = 0.0;
    //Distance between front and back wheels
    public static final double kWheelBase = 0.0;
  
    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2 ));

      /*
       * Kinematics order:
       * 1. Mod0
       * 2. Mod1
       * 3. Mod2
       * 4. Mod3
       */

    /*Free speed of each gearing:
    * 5.50 = 18.01 ft/s
    * 6.55 = 15.12 ft/s
    */
    public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(18.01);
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = 
                kPhysicalMaxSpeedMetersPerSecond / 4; //TODO: TeleOp drive speed
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4; //TODO: TeleOp angle speed
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    public static final double kDriveBaseRadius = 0.0;

    public static final double traslationP = 0.0,
                               traslationD = 0.0,
                               rotationP = 0.0,
                               rotationD = 0.0;
  }

  /*************** MISC ****************/

  public static final class VisionConstants {

    public static final String tapeLimelight = "limelight-tape";
    public static final String tagLimelightName = "limelight-tags";

    public static double HEIGHT_OF_HIGH_NODE = 0.90; //Elevation of Target
    public static double HEIGHT_OF_MID_NODE = 0.60;
    public static double HEIGHT_OF_TAG = 0.45;
    public static double LIMELIGHT_FLOOR_CLEREANCE= 0.04819; //Elevation of the Limelight
    public static double LIMELIGHT_VERTICAL_ANGLE = 0;

    public static final int normalTracking_Pipeline = 0,
                            lowAlign_Pipeline = 1,
                            midAlign_Pipeline = 2,
                            highAlign_Pipeline = 3;

  }

  public static final class IOConstants{
    public static final double kDeadband = 0.05;
  }
}
