// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.DocFlavor.STRING;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  /*private final DutyCycleEncoder m_encoder =
      new DutyCycleEncoder(ArmConstants.ABSOLUTE_ENCODER_PORT);*/

  private final CANcoder m_encoder = new CANcoder(ArmConstants.ABSOLUTE_ENCODER_ID, "rio");

  CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kS, ArmConstants.kG,
          ArmConstants.kV, ArmConstants.kA);

          private TrapezoidProfile.State m_tpState = new TrapezoidProfile.State(0.0, 0.0);

  static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> 
    kDistanceToArmAngle = new InterpolatingTreeMap<>();

  static{ //Added offset of 5 degrees of all angles because of mechanical problems
    kDistanceToArmAngle.put(new InterpolatingDouble(1.66),  new InterpolatingDouble(160.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(2.05),  new InterpolatingDouble(153.0)); 
    kDistanceToArmAngle.put(new InterpolatingDouble(2.66),  new InterpolatingDouble(143.5));
    kDistanceToArmAngle.put(new InterpolatingDouble(3.50),  new InterpolatingDouble(138.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(4.15),  new InterpolatingDouble(135.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(4.35),  new InterpolatingDouble(134.0));
  }

  private SendableChooser<String> armModeChooser = new SendableChooser<>();
  private String currentModeSelection;
  private final String[] modeNames = {"BRAKE", "COAST"};

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
    //Check encoder direction
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    encoderConfig.MagnetSensor.MagnetOffset = 0.0;

    m_encoder.getPosition().setUpdateFrequency(100);

    m_encoder.getConfigurator().apply(encoderConfig);
    //m_encoder.setPosition(0.4);
    
    // Start arm at rest in neutral position
    setGoal(90.3);

    //   this.m_controller.setIZone(20);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(true);
    rightMotor.follow(leftMotor, true);

    leftMotor.setSmartCurrentLimit(80);
    rightMotor.setSmartCurrentLimit(80);

    leftMotor.setCANTimeout(0);
    rightMotor.setCANTimeout(0);

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);//change do break

    SmartDashboard.putNumber("Arm kP", akP);
    SmartDashboard.putNumber("Arm kI", akI);
    SmartDashboard.putNumber("Arm kD",akD);

    armModeChooser.setDefaultOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Coast Mode", modeNames[1]);

    SmartDashboard.putData("Arm Mode", armModeChooser);
  }

  @Override
  public void periodic() {
      super.periodic();

      SmartDashboard.putNumber("Arm Angle", getMeasurement());

      SmartDashboard.putBoolean("Over Angle", overAngle());

      SmartDashboard.putBoolean("ArmEnabled", this.m_enabled);

      SmartDashboard.putNumber("Arm SetPoint Pos", this.getController().getSetpoint().position);

      SmartDashboard.putNumber("Arm Pose Error", this.getController().getPositionError());


      if(DriverStation.isDisabled()){
        currentModeSelection = armModeChooser.getSelected();
        switch (currentModeSelection) {
          case "BRAKE":
            armSetBrake();
          break;

          case "COAST":
            armSetCoast();
          break;
        }
      }

      double akP = SmartDashboard.getNumber("Arm kP", ArmConstants.kP),
             akI = SmartDashboard.getNumber("Arm kI", ArmConstants.kI),
             akD = SmartDashboard.getNumber("Arm kD", ArmConstants.kD);

      if (ArmConstants.kP != akP) {ArmConstants.kP = akP; getController().setP(akP);}
      if (ArmConstants.kI != akI) {ArmConstants.kI = akI; getController().setI(akI);}
      if (ArmConstants.kD != akD) {ArmConstants.kD = akD; getController().setD(akD);}
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
    return (m_encoder.getAbsolutePosition().getValueAsDouble() * 360) - 153;
  }

  public double getAngleForDistance(double distance){
    return kDistanceToArmAngle.getInterpolated(
      new InterpolatingDouble(
        Math.max(Math.min(distance, 4.35 ), 1.66))).value;
  }

  public Command goToPosition(double position){
    Command ejecutable = Commands.runOnce(
                () -> {
                  this.getController().reset(getMeasurement());
                  this.setGoal(position);
                  this.enable();
                },
                this);
    return ejecutable;
  }

  public boolean overAngle(){
    if(this.m_enabled && (getMeasurement() > 181 || getMeasurement() < 89)){
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
  
  public void updateArmSetpoint(double setpoint){
    m_tpState.position = Units.degreesToRadians(setpoint);
    setGoal(setpoint);
  }

  public void armSetCoast(){
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);  
  }

  public void armSetBrake(){
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);  
  }
}