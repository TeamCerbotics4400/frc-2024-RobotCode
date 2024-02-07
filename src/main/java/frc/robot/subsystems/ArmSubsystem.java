// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/** A robot arm subsystem that moves with a motion profile. **/

/*
 * Why not use Rev's motion profiling if we have Neo 500 motors?
 * Well we have an Absolute Encoder that returns the current Arm angle but is not connected to the
 * Spark Max data port but to the RoboRIO DIO port 2, so we decided that it was better to use the
 * ProfiedPIDSubsystem class for our Arm so we can have a Motion Profiling PID with the absolute
 * Encoder as a feedback Device.
 */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_ARM_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_ARM_ID, MotorType.kBrushless);

  private final DutyCycleEncoder m_encoder =
      new DutyCycleEncoder(ArmConstants.ABSOLUTE_ENCODER_PORT);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kS, ArmConstants.kG,
          ArmConstants.kV, ArmConstants.kA);

      double akP = 0, akD = 0;

  private static final Translation2d rootPosition = new Translation2d(0.0, 0.0);

  boolean onTarget;

  DoubleArrayLogEntry arm3dPose;

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            ArmConstants.kD,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationMetersPerSecondSquared)),
        90.3);
    
    //Makes the Arm absolute Encoder return every rotation as angles
    m_encoder.setDistancePerRotation(360.0);
    // Start arm at rest in neutral position
    setGoal(90.3);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(true);
    rightMotor.follow(leftMotor, true);

    leftMotor.setSmartCurrentLimit(80);
    rightMotor.setSmartCurrentLimit(80);

    leftMotor.setCANTimeout(0);
    rightMotor.setCANTimeout(0);

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putNumber("Arm kP", akP);
    SmartDashboard.putNumber("Arm kD",akD);
  }

  @Override
  public void periodic() {
      super.periodic();

      SmartDashboard.putNumber("Arm Angle", getMeasurement());

      //SmartDashboard.putBoolean("Arm ready", isReady());
      //SmartDashboard.putBoolean("Is Intaking Pose", isInIntakingPos());

      /*double akP = SmartDashboard.getNumber("Arm kP", 0.69),
             akD = SmartDashboard.getNumber("Arm kD", 0.0001);

      if (ArmConstants.kP != akP) {ArmConstants.kP = akP; getController().setP(akP);}
      if (ArmConstants.kD != akD) {ArmConstants.kD = akD; getController().setD(akD);}*/

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    leftMotor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    //Minus 70.5 because that gives us a range betwueen 0-180 degrees, 0 being the left position
    //and 180 the right position while 90 degrees is the idle vertical position
    return m_encoder.getDistance() - 136.0;
  }

  public Command goToPosition(double position){
    Command ejecutable = Commands.runOnce(
                () -> {
                  this.setGoal(position);
                  this.enable();
                },
                this);
    return ejecutable;
  }

  public Pose3d getSuperStructurePose(){
    return new Pose3d(0,0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  }

  public Pose3d getArm3dPose(){
    return new Pose3d(rootPosition.getX(), 0.0, rootPosition.getY(), 
                          new Rotation3d(0.0, getMeasurement(), 0.0));
  }

  public double[] posetoArray(Pose3d pose){
    return new double[] {pose.getX(), pose.getY(), pose.getZ(), 
      pose.getRotation().getX(), pose.getRotation().getY(), pose.getRotation().getZ()};
  }

  //For use in autonomous methods to shoot after the Arm is in position
  public boolean isWithinThreshold(double value, double target, double threshold){
    return Math.abs(value - target) < threshold;
  }

  public boolean isInPosition(){
    return isWithinThreshold(getMeasurement(), getController().getGoal().position, ArmConstants.ARM_THRESHOLD);
  }
}