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
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final CANSparkMax climberMotor = 
                new CANSparkMax(ClimberConstants.CLIMBER_ID, MotorType.kBrushless);
  private final SparkPIDController climberPIDController;
  private final RelativeEncoder climberEncoder;

  private double extendedClimber = 0.1;   //Test optimal height to the chain
  private double retractedClimber = 0.1;

  public ClimberSubsystem() {
    climberMotor.clearFaults();

    climberMotor.restoreFactoryDefaults();

    climberMotor.setInverted(false);

    climberMotor.setIdleMode(IdleMode.kBrake);

    climberPIDController = climberMotor.getPIDController();
    climberEncoder = climberMotor.getEncoder();

    climberPIDController.setP(ClimberConstants.kP);
    climberPIDController.setI(ClimberConstants.kI);
    climberPIDController.setD(ClimberConstants.kD);
    climberPIDController.setFF(ClimberConstants.kFF);

    //climberEncoder.setPositionConversionFactor(ClimberConstants.CLIMBER_GEARBOX);

    /*setSoftLimits();
    enableSoftLimit(); */
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position",getClimberPosition()); 
  }

  public double getClimberPosition(){
    return climberEncoder.getPosition();
  }

  public void ExtendClimber(){
    climberMotor.set(0.5);
  }

  public void RetractClimber(){
    climberMotor.set(-0.5);
  }

  public void stopClimber(){
    climberMotor.stopMotor();
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
