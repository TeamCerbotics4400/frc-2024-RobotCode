// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final CANSparkMax climberMotor = 
                new CANSparkMax(ClimberConstants.CLIMBER_ID, MotorType.kBrushless);
  private final SparkPIDController climberPIDController;
  private final RelativeEncoder climberEncoder;

  private double extendedClimber = 2133.37158203125;   //2133.37158203125 HIGHEST VALUE || second 1551.4813232421875 highest value
  private double retractedClimber = 4074.0712890625;   //4074.0712890625   lowest value

  double leftkP = 0.0, leftkI = 0.0, leftkD = 0.0, leftkFF = 0.0;
                                                    // 5 vueltas
  private double openExtendedClimber = 1.0;  

  public ClimberSubsystem() {
    climberMotor.clearFaults();

    climberMotor.restoreFactoryDefaults();

    climberMotor.setInverted(true);

    climberMotor.setIdleMode(IdleMode.kBrake);

    climberPIDController = climberMotor.getPIDController();
    climberEncoder = climberMotor.getEncoder();

    climberPIDController.setP(ClimberConstants.kP);
    climberPIDController.setI(ClimberConstants.kI);
    climberPIDController.setD(ClimberConstants.kD);

    climberEncoder.setPositionConversionFactor(ClimberConstants.CLIMBER_GEARBOX);

    resetEncoder();


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position",getClimberPosition()); 

  }


  public double getClimberPosition(){
    return climberEncoder.getPosition();
  }

  public void extendClimber(){
    climberPIDController.setReference(2100.0, ControlType.kPosition);
  }

  public void retractClimber(){
    climberPIDController.setReference(4070.0, ControlType.kPosition);
  }

  public void openLoopExtend(){
    climberMotor.set(openExtendedClimber);
  }

  public void stopClimber(){
    climberMotor.stopMotor();
  }

  public void resetEncoder(){
    climberEncoder.setPosition(0);
  }

  public boolean isWithinThreshold(double value, double target, double threshold){
    return Math.abs(value - target) < threshold;
  }

  public boolean isInPosition(double position){
    return isWithinThreshold(getClimberPosition(), position, ClimberConstants.CLIMBER_THRESHOLD);
  }
  /*
  public void setSoftLimits(){
    //Check Encoder Max Pos
    climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 8); //Test the limits
    climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);   
  }
  public void enableSoftLimit(){
    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);    
  } */
}
