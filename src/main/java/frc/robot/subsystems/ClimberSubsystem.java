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
      private final CANSparkMax climberMotor = new CANSparkMax(ClimberConstants.CLIMBER_ID,MotorType.kBrushless);
      private final SparkPIDController climberPIDController;
      private final RelativeEncoder climberEncoder;
      private double climberSetPoint = 0;
      private double climberSpeed = 0;
      private double descendSetPoint = 0;
      private double descendSpeed = 0;

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  

         double cRPM = SmartDashboard.getNumber("Climber RPM", 0);

    if (climberSetPoint != cRPM){climberSetPoint = cRPM;}
    if (descendSetPoint != cRPM){descendSetPoint= cRPM*-1;}

    SmartDashboard.putNumber("Current Upper RPM",climberEncoder.getVelocity());

  }

  public void setClimbingSpeed(){
    climberSpeed = climberSetPoint;
    climberPIDController.setReference(climberSpeed,ControlType.kVelocity);
  }
  public void setDescendingSpeed(){
    descendSpeed = descendSetPoint;
    climberPIDController.setReference(descendSpeed,ControlType.kVelocity);

  }
  
  public void stopClimber(){
    climberMotor.set(0.0);
  }
}
