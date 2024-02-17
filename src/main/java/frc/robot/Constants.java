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
    public static  double kP = 0.12,
                               kI = 0.001,
                               kD = 0,
                               kFF = 0.0,
                               kS = 0.12858,
                               kV = 0.41582,
                               kA = 0.10791;
    public static double kPTurning = 0.5;
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
     *     listo           B        listo
     */

     //Offsets are different in each robot and encoder;
    public static final class Module0{ 
      public static final int DRIVE_ID = 2;
      public static final int TURN_ID = 1;
      public static final boolean driveReversed = true;
      public static final boolean turnReversed = true;
      public static final int ABSOLUTE_ID = 3; 
      public static double encoderOffset = -74.90;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final class Module1{
      public static final int DRIVE_ID = 4;
      public static final int TURN_ID = 3;
      public static final boolean driveReversed = false;
      public static final boolean turnReversed = true;
      public static final int ABSOLUTE_ID = 0;
      public static double encoderOffset = 62.2;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final class Module2{
      public static final int DRIVE_ID = 6;
      public static final int TURN_ID = 5;
      public static final boolean driveReversed = false;
      public static final boolean turnReversed = true;
      public static final int ABSOLUTE_ID = 1;
      public static double encoderOffset = -168.6;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final class Module3{
      public static final int DRIVE_ID = 7;
      public static final int TURN_ID = 8;
      public static final boolean driveReversed = true;
      public static final boolean turnReversed = true;
      public static final int ABSOLUTE_ID = 2;
      public static double encoderOffset = 180.4;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, ABSOLUTE_ID, encoderOffset);
    }

    public static final int IMU_ID = 15;

    //Distance between left and right wheels
    public static final double kTrackWidth = 0.6096;
    //Distance between front and back wheels
    public static final double kWheelBase = 0.635;
  
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
                kPhysicalMaxSpeedMetersPerSecond * 0.80; //TODO: TeleOp drive speed
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond / 3; //TODO: TeleOp angle speed
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    public static final double kDriveBaseRadius = 0.30;

    public static final double traslationP = 0.15,
                               traslationD = 0.0,
                               rotationP = 0.5,
                               rotationD = 0.0;
  }

  public static final class ShooterConstants {
    public static byte LEFT_MOTOR_ID = 14;
    public static byte RIGHT_MOTOR_ID = 13;

    public static double lkP = 0.02,     //For low rpm: 0.046
                         lkI = 0,         // 0
                         lkD = 0.00019,    //0.0003
                         lkFF = 0.112,    //0.128
                         rkP = 0.01,     //0.004
                         rkI = 0,         //0
                         rkD = 0.00019,    //0.0005
                         rkFF = 0.112;   //0.1175

    public static double SHOOTER_THRESHOLD = 150;            
  }

  public static final class IntakeConstants {
    public static byte INTAKE_ID = 12;
  }

  public static final class ArmConstants {
    public static byte LEFT_ARM_ID = 9;   //9
    public static byte RIGHT_ARM_ID = 10;  //10

    public static int ABSOLUTE_ENCODER_PORT = 0;

    public static double ARM_GEARBOX = 320.0 / 1.0;

    public static double kP = 0.32,
                         kI = 0.42,
                         kD = 0.0039,
                         kFF = 0.0,
                         kMaxVelocityRadPerSecond = 500,
                         kMaxAccelerationMetersPerSecondSquared = 550,
                         kS = 0.013804,
                         kV = 0.00028699,
                         kA = 0.00052411,
                         kG = 0.93532;
    
    public static double INTAKING_POSITION = 180.5;
    public static double SPEAKER_SCORING_POSITION = 160.0;//115;
    public static double INIT_POSITION = 95.0;
    public static double IDLE_UNDER_STAGE = 160.0;
    
    public static double ARM_THRESHOLD = 1.5;

    public static final byte TELESCOPE_ID = 11;
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

    public static final String neuralLimelight = "limelight-neural";
    public static final String tagLimelightName = "limelight-tags";

    public static final int main_Pipeline = 0,
                            far_Pipeline = 1;
  }

  public static final class IOConstants{
    public static final int DRIVER_ONE_PORT = 0;
    public static final int DRIVER_TWO_PORT = 1;
    public static final double kDeadband = 0.05;
  }
}
