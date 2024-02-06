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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final CANSparkMax climberMotor = 
                new CANSparkMax(ClimberConstants.CLIMBER_ID,MotorType.kBrushless);
  private final SparkPIDController climberPIDController;
  private final RelativeEncoder climberEncoder;

  private double extendedClimber = 0.0;
  private double retractedClimber = 0.0;

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

    climberEncoder.setPositionConversionFactor(ClimberConstants.CLIMBER_GEARBOX);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Climber Position",getClimberPosition()); //for now, just to make space on the shuffleboard
  }

  public double getClimberPosition(){
    return climberEncoder.getPosition();
  }

  public void ExtendClimber(){
    climberPIDController.setReference(extendedClimber, ControlType.kPosition);
  }

  public void RetractClimber(){
    climberPIDController.setReference(retractedClimber, ControlType.kPosition);
  }

  public void stopClimber(){
    climberMotor.stopMotor();
  }
}
