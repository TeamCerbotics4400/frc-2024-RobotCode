// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

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

public final class Constants {
  public static boolean needToLog = true;

  public static final class DriveConstants{
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


    public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    public static final double MaxAngularRate = 1.5 * Math.PI;

    public static final double traslationP = 5.0,//0.5,
                               traslationD = 0.0,
                               rotationP = 5.0, //0.2
                               rotationD = 0.0;
  }

    public static class FieldConstants {
      public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), new Rotation2d(0));
      public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));

      public static final double NOTE_DIAMETER = 14; // Outer diameter of note

  }

  public static final class ShooterConstants {
    public static byte UPPER_MOTOR_ID = 13;   //CHANGED   14
    public static byte LOWER_MOTOR_ID = 14;   //13

    public static final double kTimeToShoot = .13; //Time it takes before note leave the shooter

    public static double ukP = 1.2,     //For low rpm: 0.046
                         ukI = 0,         // 0
                         ukD = 0.0,    //0.0003
                         ukS = 0.50189,
                         ukV = 0.0016926,
                         ukA = 0.00059492,    //0.128
                         lkP = 1.2,     //0.004
                         lkI = 0,         //0
                         lkD = 0.0,    //0.0005
                         lkS = 0.50189,
                         lkV = 0.0016926,
                         lkA = 0.00059492;

    public static double SHOOTER_THRESHOLD = 150;            
  }

  public static final class IntakeConstants {
    public static byte INTAKE_ID = 12;

    public static double INTAKE_ANGLE = 181.0;
  }

  public static final class ArmConstants {
    public static byte LEFT_ARM_ID = 9;   //9
    public static byte RIGHT_ARM_ID = 10;  //10

    //Changed to CANCoder
    public static int ABSOLUTE_ENCODER_ID = 17;

    public static double ARM_GEARBOX = 320.0 / 1.0;

    public static double kP = 0.32,//0.25,
                         kI = 0.42,//0.03,
                         kD = 0.0055,
                         kFF = 0.0,
                         kMaxVelocityRadPerSecond = 1000,//500, 
                         kMaxAccelerationMetersPerSecondSquared = 1000,//500, 
                         kS = 0.013804,
                         kV = 0.00028699,
                         kA = 0.00052411,
                         kG = 0.93532;
    
    public static double INTAKING_POSITION = 180.5;
    public static double SPEAKER_SCORING_POSITION = 160.0;//115;
    public static double INIT_POSITION = 95.0;
    public static double IDLE_UNDER_STAGE = 170.0;
    
    public static double ARM_THRESHOLD = 1.5;
  }

  public static final class ClimberConstants{
    public static byte CLIMBER_ID = 16; 

    public static double kP = 0.5,
                         kI = 0,
                         kD = 1,
                         kMaxVelocityLinearPerSecond = 0,
                         kMaxAccelerationMetersPerSecondSquared = 1;

    public static double CLIMBER_GEARBOX = 20.0 / 1.0;

    public static double CLIMBER_THRESHOLD = 0.35;
  }

 
  /*************** MISC ****************/

  public static final class VisionConstants {

    public static final String neuralLimelight = "limelight-neural";
    public static final String tagLimelight = "limelight-tags";

    public static final int main_Pipeline = 0,
                            far_Pipeline = 1,
                              close_Pipeline = 2,
                            autoAim_Pipeline = 3;
  }

  public static final class IOConstants{
    public static final int DRIVER_ONE_PORT = 0;
    public static final int DRIVER_TWO_PORT = 1;
    public static final double kDeadband = 0.05;
  }
}