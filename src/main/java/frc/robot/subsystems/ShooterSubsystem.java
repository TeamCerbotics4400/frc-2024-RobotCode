// Copylower (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX upperFlyWheel = new TalonFX(ShooterConstants.UPPER_MOTOR_ID, "rio"); //14  upper-upper
  TalonFX lowerFlyWheel = new TalonFX(ShooterConstants.LOWER_MOTOR_ID, "rio");  //13   down-lower
  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  
  TalonFXConfiguration upperConfig = new TalonFXConfiguration();
  TalonFXConfiguration lowerConfig = new TalonFXConfiguration();

  private final VelocityVoltage upperVelocity = new VelocityVoltage(0);
  private final VelocityVoltage lowerVelocity = new VelocityVoltage(0);

  double upperSetPoint = 1000, lowerSetPoint = 1000;
  double pkP = 0, pKd = 0;


  public ShooterSubsystem() {
    upperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  
    lowerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

//Put kbrake
    upperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    lowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    upperConfig.Slot0.kP = ShooterConstants.lkP;
    upperConfig.Slot0.kI = ShooterConstants.lkI;
    upperConfig.Slot0.kD = ShooterConstants.lkD; 
    upperConfig.Slot0.kV = ShooterConstants.lkFF; 
    lowerConfig.Slot0.kP = ShooterConstants.rkP; 
    lowerConfig.Slot0.kI = ShooterConstants.rkI;
    lowerConfig.Slot0.kD = ShooterConstants.rkD; 
    lowerConfig.Slot0.kV = ShooterConstants.rkFF;
  
    upperFlyWheel.getConfigurator().apply(upperConfig);
    lowerFlyWheel.getConfigurator().apply(lowerConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     /*  double uRPM = SmartDashboard.getNumber("upper RPM", 0),
             lRPM = SmartDashboard.getNumber("lower RPM", 0);

    if (upperSetPoint != uRPM){upperSetPoint = uRPM;}
    if (lowerSetPoint != lRPM){lowerSetPoint = lRPM;}*/

    SmartDashboard.putNumber("Current shooter rpm", getRPM());
    SmartDashboard.putNumber("Current voltage",getVoltage());

    /*SmartDashboard.putNumber("upper RPM", upperSetPoint);
    SmartDashboard.putNumber("lower RPM", lowerSetPoint);*/
  }
  
  public void setupperSpeed(double setPoint){
    upperVelocity.Velocity = setPoint / 60;
    upperFlyWheel.setControl(upperVelocity);
  }

  public void setlowerSpeed(double setPoint){
    lowerVelocity.Velocity = setPoint / 60;
    lowerFlyWheel.setControl(lowerVelocity);
  }

  public void setBothSpeeds(){
    setlowerSpeed(lowerSetPoint);
    setupperSpeed(upperSetPoint);
  }
  public void stopupper(){
    upperFlyWheel.set(0);
  }

  public void stoplower(){
    lowerFlyWheel.set(0);
  }

  public double getRPM(){
    return lowerFlyWheel.getVelocity().getValueAsDouble() * 60;
  }

  public boolean isWithinThreshold(double value, double target, double threshold){
    return Math.abs(value - target) < threshold;
  }

  public boolean isInRPMS(){
    return isWithinThreshold(
      getRPM(), 
      upperFlyWheel.getClosedLoopReference().getValue(), 
      ShooterConstants.SHOOTER_THRESHOLD);
  }
  public double getVoltage(){
    return filter.calculate(lowerFlyWheel.getSupplyCurrent().getValueAsDouble());
  }

}
