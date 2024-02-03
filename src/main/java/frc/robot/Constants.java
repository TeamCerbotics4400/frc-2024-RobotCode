// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 5.50; //Drive Gear Ratio, 5.50 or 6.55
    public static final double kTurningMotorGearRatio = 1 / 10.29; //Turning Gear Ratio
    public static final double kDriveEncoderRot2Meter = 
                                kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kP = 0.0,
                               kI = 0,
                               kD = 0,
                               kFF = 0.0,
                               kS = 0.0,
                               kV = 0.0,
                               kA = 0.0;
    public static final double kPTurning = 0.5;
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
      public static final boolean driveReversed = true;
      public static final boolean turnReversed = true;
      public static final int ABSOLUTE_ID = 1; 
      public static double encoderOffset = 0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final class Module1{
      public static final int DRIVE_ID = 3;
      public static final int TURN_ID = 4;
      public static final boolean driveReversed = true;
      public static final boolean turnReversed = true;
      public static final int ABSOLUTE_ID = 2;
      public static double encoderOffset = 0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final class Module2{
      public static final int DRIVE_ID = 5;
      public static final int TURN_ID = 6;
      public static final boolean driveReversed = false;
      public static final boolean turnReversed = true;
      public static final int ABSOLUTE_ID = 3;
      public static double encoderOffset = 0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final class Module3{
      public static final int DRIVE_ID = 7;
      public static final int TURN_ID = 8;
      public static final boolean driveReversed = false;
      public static final boolean turnReversed = true;
      public static final int ABSOLUTE_ID = 4;
      public static double encoderOffset = 0;

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

  public static final class ShooterConstants {
    public static byte LEFT_MOTOR_ID = 0;
    public static byte RIGHT_MOTOR_ID = 0;

    public static double kP = 0.02,
                         kI = 0,
                         kD = 0.000190,
                         kFF = 0.144;

  }

  public static final class IntakeConstants {
    public static byte INTAKE_ID = 0;
  }

  public static final class ArmConstants {
    public static byte LEFT_ARM_ID = 0; 
    public static byte RIGHT_ARM_ID = 0;

    public static double ARM_GEARBOX = 320.0 / 1.0;

    public static double kP = 0.13821,//0.0011773,//0.012904,//4.3755E-09,
                         kI = 0.0,
                         kD = 0.021835,//0.00017643,//0.0024401,//8.274E-10,
                         kFF = 0.0,//0.000156,
                         kMaxVelocityRadPerSecond = 320,
                         kMaxAccelerationMetersPerSecondSquared = 250,//320,
                         kS = 0.94615,//0.82172,
                         kV = 0.0021715,//0.0047927,
                         kA = 0.0019641,//0.003212,
                         kG = 0.12588;//0.44033;

    public static double IDLE_POSITION = 90.0;
    
    public static double SUBSTATION_POSITION = 130.5;
    public static double SCORING_POSITION = 100;//115;
    public static double BACK_FLOOR_POSITION = 1.15;
    public static double FRONT_FLOOR_POSITION = 175.0;
    public static double AVE_MARIA_SHOOT_POSITION = 120.0;
    public static double COUNTER_BALANCE_POSITION = 60.0;
    public static double TESTING_ANGLE = 90.0;
    public static double ARM_THRESHOLD = 9.5;

    public static final byte TELESCOPE_ID = 0;
    public static final int TELESCOPING_GEAR_RATIO = 0;
    public static double kTP = 0,
                         kTI = 0,
                         kTD = 0,
                         kTFF = 0; 
  }

  public static final class ClimberConstants{
    public static byte CLIMBER_ID = 0; 

    public static double kP = 0.03,
                         kI = 0,
                         kD = 0,
                         kFF = 0.1,
                         kMaxVelocityLinearPerSecond = 0,
                         kMaxAccelerationMetersPerSecondSquared = 0;

    public static double CLIMBER_GEARBOX = 0.0;
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
