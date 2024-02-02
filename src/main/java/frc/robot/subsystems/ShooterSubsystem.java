// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX upperFlyWheel = new TalonFX(ShooterConstants.LEFT_MOTOR_ID, "rio");
  TalonFX lowerFlyWheel = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID, "rio");

  TalonFXConfiguration upperConfig = new TalonFXConfiguration();
  TalonFXConfiguration lowerConfig = new TalonFXConfiguration();

  private final VelocityVoltage upperVelocity = new VelocityVoltage(0);
  private final VelocityVoltage lowerVelocity = new VelocityVoltage(0);

  double upperSetPoint = 0, lowerSetPoint = 0;

  public ShooterSubsystem() {
    upperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  //Chekc later motor direction
    lowerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    upperConfig.Slot0.kP = ShooterConstants.kP;
    upperConfig.Slot0.kI = ShooterConstants.kI;
    upperConfig.Slot0.kD = ShooterConstants.kD; 
    upperConfig.Slot0.kV = ShooterConstants.kFF; 
    lowerConfig.Slot0.kP = ShooterConstants.kP; 
    lowerConfig.Slot0.kI = ShooterConstants.kI;
    lowerConfig.Slot0.kD = ShooterConstants.kD; 
    lowerConfig.Slot0.kV = ShooterConstants.kFF;
  
    upperFlyWheel.getConfigurator().apply(upperConfig);
    lowerFlyWheel.getConfigurator().apply(lowerConfig);

    SmartDashboard.putNumber("Upper RPM", upperSetPoint);   SmartDashboard.putNumber("Lower RPM", lowerSetPoint);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


      double uRPM = SmartDashboard.getNumber("Upper RPM", 0),
             lRPM = SmartDashboard.getNumber("Lower RPM", 0);

    if (upperSetPoint != uRPM){upperSetPoint = uRPM;}
    if (lowerSetPoint != lRPM){lowerSetPoint = lRPM;}

    SmartDashboard.putNumber("Current Upper RPM", upperFlyWheel.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Current Lower RPM", lowerFlyWheel.getVelocity().getValueAsDouble() * 60);

  }
  
  public void setUpperSpeed(){
    upperVelocity.Velocity = upperSetPoint / 60;
    upperFlyWheel.setControl(upperVelocity);
  }
  public void setLowerSpeed(){
    lowerVelocity.Velocity = lowerSetPoint / 60;
    lowerFlyWheel.setControl(lowerVelocity);
  }
  public void stopUpper(){
    upperFlyWheel.set(0);
  }

  public void stopLower(){
    lowerFlyWheel.set(0);
  }
  public double getRPM(){
    return lowerFlyWheel.getVelocity().getValueAsDouble()*60;
  }
}
