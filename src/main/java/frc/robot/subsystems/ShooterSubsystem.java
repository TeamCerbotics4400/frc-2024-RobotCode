// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX leftFlyWheel = new TalonFX(ShooterConstants.LEFT_MOTOR_ID, "rio"); //14  upper-left
  TalonFX rightFlyWheel = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID, "rio");  //13   down-RIGHT
  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);


  TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration rightConfig = new TalonFXConfiguration();

  private final VelocityVoltage leftVelocity = new VelocityVoltage(0);
  private final VelocityVoltage rightVelocity = new VelocityVoltage(0);

  double leftSetPoint = 1000, rightSetPoint = 1000;
  double pkP = 0, pKd = 0;


  public ShooterSubsystem() {
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leftConfig.Slot0.kP = ShooterConstants.lkP;
    leftConfig.Slot0.kI = ShooterConstants.lkI;
    leftConfig.Slot0.kD = ShooterConstants.lkD; 
    leftConfig.Slot0.kV = ShooterConstants.lkFF; 
    rightConfig.Slot0.kP = ShooterConstants.rkP; 
    rightConfig.Slot0.kI = ShooterConstants.rkI;
    rightConfig.Slot0.kD = ShooterConstants.rkD; 
    rightConfig.Slot0.kV = ShooterConstants.rkFF;
  
    leftFlyWheel.getConfigurator().apply(leftConfig);
    rightFlyWheel.getConfigurator().apply(rightConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     /*  double uRPM = SmartDashboard.getNumber("left RPM", 0),
             lRPM = SmartDashboard.getNumber("right RPM", 0);

    if (leftSetPoint != uRPM){leftSetPoint = uRPM;}
    if (rightSetPoint != lRPM){rightSetPoint = lRPM;}*/

    SmartDashboard.putNumber("Current left RPM", leftFlyWheel.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Current right RPM", rightFlyWheel.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Current voltage",getVoltage());

    /*SmartDashboard.putNumber("left RPM", leftSetPoint);
    SmartDashboard.putNumber("right RPM", rightSetPoint);*/
  }
  
  public void setleftSpeed(double setPoint){
    leftVelocity.Velocity = setPoint / 60;
    leftFlyWheel.setControl(leftVelocity);
  }

  public void setrightSpeed(double setPoint){
    rightVelocity.Velocity = setPoint / 60;
    rightFlyWheel.setControl(rightVelocity);
  }

  public void stopleft(){
    leftFlyWheel.set(0);
  }

  public void stopright(){
    rightFlyWheel.set(0);
  }

  public double getRPM(){
    return rightFlyWheel.getVelocity().getValueAsDouble() * 60;
  }

  public boolean isWithinThreshold(double value, double target, double threshold){
    return Math.abs(value - target) < threshold;
  }

  public boolean isInRPMS(){
    return isWithinThreshold(
      getRPM(), 
      leftFlyWheel.getClosedLoopReference().getValue(), 
      ShooterConstants.SHOOTER_THRESHOLD);
  }
  public double getVoltage(){
    return filter.calculate(rightFlyWheel.getSupplyCurrent().getValueAsDouble());
  }
}
