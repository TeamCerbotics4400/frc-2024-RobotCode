// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import team4400.Util.Interpolation.InterpolatingDouble;
import team4400.Util.Interpolation.InterpolatingTreeMap;
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

  static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> 
    kDistanceToArmAngle = new InterpolatingTreeMap<>();

  static{
    kDistanceToArmAngle.put(new InterpolatingDouble(1.56),  new InterpolatingDouble(165.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(1.75), new InterpolatingDouble(150.0));
    
  }

  boolean onTarget;

  double akP = 0.32, akI = 0.42, akD = 0.0039;

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            ArmConstants.kI,
            ArmConstants.kD,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationMetersPerSecondSquared)),
        95.0);
    
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
    SmartDashboard.putNumber("Arm kI", akI);
    SmartDashboard.putNumber("Arm kD",akD);
  }

  @Override
  public void periodic() {
      super.periodic();

      SmartDashboard.putNumber("Arm Angle", getMeasurement());

      SmartDashboard.putBoolean("Over Angle", overAngle());

      SmartDashboard.putBoolean("ArmEnabled", this.m_enabled);

      SmartDashboard.putNumber("Arm SetPoint Pos", this.getController().getSetpoint().position);

      SmartDashboard.putNumber("Arm Pose Error", this.getController().getPositionError());

      SmartDashboard.putBoolean("Is in position", isInPosition());

      SmartDashboard.putBoolean("Has Left Controller Reset", hasLeftArmReset());
      SmartDashboard.putBoolean("Has Right Controller Reset", hasRightArmReset());

      overAngle();

      safetyDisable();

      double akP = SmartDashboard.getNumber("Arm kP", ArmConstants.kP),
             akI = SmartDashboard.getNumber("Arm kI", ArmConstants.kI),
             akD = SmartDashboard.getNumber("Arm kD", ArmConstants.kD);

      if (ArmConstants.kP != akP) {ArmConstants.kP = akP; getController().setP(akP);}
      if (ArmConstants.kI != akI) {ArmConstants.kI = akI; getController().setI(akI);}
      if (ArmConstants.kD != akD) {ArmConstants.kD = akD; getController().setD(akD);}


      /* 
       if (DriverStation.isDisabled()){
        rightMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setIdleMode(IdleMode.kCoast);

       }
       else {
        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setIdleMode(IdleMode.kBrake);
       }
       */
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

  public boolean overAngle(){
    if(this.m_enabled && (getMeasurement() > 184 || getMeasurement() < 90)){
      return true;
    } else {
      return false;
    }
  }

  public void safetyDisable(){
    if(overAngle()){
      this.disable();
    }
  }

  //For use in autonomous methods to shoot after the Arm is in position
  public boolean isWithinThreshold(double value, double target, double threshold){
    return Math.abs(value - target) < threshold;
  }

  public boolean isInPosition(){
    return isWithinThreshold(getMeasurement(), getController().getGoal().position, ArmConstants.ARM_THRESHOLD);
  }

  public boolean hasLeftArmReset(){
    return leftMotor.getStickyFault(FaultID.kHasReset);
  }

  public boolean hasRightArmReset(){
    return rightMotor.getStickyFault(FaultID.kHasReset);
  }
}