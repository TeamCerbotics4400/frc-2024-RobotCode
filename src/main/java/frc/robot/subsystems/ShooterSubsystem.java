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
 private final TalonFX upperFlyWheel = new TalonFX(ShooterConstants.UPPER_MOTOR_ID, "rio"); 
 private final TalonFX lowerFlyWheel = new TalonFX(ShooterConstants.LOWER_MOTOR_ID, "rio");  
  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  
  TalonFXConfiguration upperConfig = new TalonFXConfiguration();
  TalonFXConfiguration lowerConfig = new TalonFXConfiguration();

  private final VelocityVoltage upperVelocity = new VelocityVoltage(0);
  private final VelocityVoltage lowerVelocity = new VelocityVoltage(0);

  /*double uKp = 0.0,
                 uKd = 0.0,
                 lKp = 0.0,
                 lKd = 0.0;*/

  public ShooterSubsystem() {
    upperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  
    lowerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    upperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    lowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //TODO: Shooter will not reach RPM but will shoot with no problems
    upperConfig.Slot0.kP = ShooterConstants.ukP;
    upperConfig.Slot0.kI = ShooterConstants.ukI;
    upperConfig.Slot0.kD = ShooterConstants.ukD; 
    upperConfig.Slot0.kS = ShooterConstants.ukS;
    upperConfig.Slot0.kV = ShooterConstants.ukV; 
    upperConfig.Slot0.kA = ShooterConstants.ukA;

    lowerConfig.Slot0.kP = ShooterConstants.lkP; 
    lowerConfig.Slot0.kI = ShooterConstants.lkI;
    lowerConfig.Slot0.kD = ShooterConstants.lkD; 
    lowerConfig.Slot0.kS = ShooterConstants.lkS;
    lowerConfig.Slot0.kV = ShooterConstants.lkV;
    lowerConfig.Slot0.kA = ShooterConstants.lkA;
  
    upperFlyWheel.getConfigurator().apply(upperConfig);
    lowerFlyWheel.getConfigurator().apply(lowerConfig);

    /*SmartDashboard.putNumber("UPPER KP", uKp);
    SmartDashboard.putNumber("Upper KD", uKd);
    SmartDashboard.putNumber("LOWER KP", lKp);
    SmartDashboard.putNumber("Lower KD", lKd);*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Current shooter rpm", getRPM());
    SmartDashboard.putNumber("Current voltage",getVoltage());

    SmartDashboard.putNumber("Lower RPM", getLowerRPM());
    SmartDashboard.putNumber("Upper RPM ", getUpperRPM());

    /*double upkp = SmartDashboard.getNumber("UPPER KP", 0.0);
    double upkd = SmartDashboard.getNumber("Upper KD", 0.0);
    double lokp = SmartDashboard.getNumber("LOWER KP", 0.0);
    double lokd = SmartDashboard.getNumber("Lower KD", 0.0);

    if(uKp != upkp){uKp = upkp; upperConfig.Slot0.kP = uKp; upperFlyWheel.getConfigurator().apply(upperConfig);}
    if(uKd != upkd){uKd = upkd; upperConfig.Slot0.kD = uKd; upperFlyWheel.getConfigurator().apply(upperConfig);}
    if(lKp != lokp){lKp = lokp; lowerConfig.Slot0.kP = lKp; lowerFlyWheel.getConfigurator().apply(lowerConfig);}
    if(lKd != lokd){lKd = lokd; lowerConfig.Slot0.kD = lKd; lowerFlyWheel.getConfigurator().apply(lowerConfig);}*/

  }
  
  public void setupperSpeed(double setPoint){
    upperVelocity.Velocity = setPoint / 60;
    upperFlyWheel.setControl(upperVelocity);
  }

  public void setlowerSpeed(double setPoint){
    lowerVelocity.Velocity = setPoint / 60;
    lowerFlyWheel.setControl(lowerVelocity);
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

  public double getLowerRPM(){
    return lowerFlyWheel.getVelocity().getValueAsDouble() * 60;
  }

  public double getUpperRPM(){
    return upperFlyWheel.getVelocity().getValueAsDouble() * 60;
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
