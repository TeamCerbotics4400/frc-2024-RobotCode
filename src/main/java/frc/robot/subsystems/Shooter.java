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

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX upperFlyWheel = new TalonFX(3, "rio");
  TalonFX lowerFlyWheel = new TalonFX(13, "rio");

  TalonFXConfiguration upperConfig = new TalonFXConfiguration();
  TalonFXConfiguration lowerConfig = new TalonFXConfiguration();

  private final VelocityVoltage upperVelocity = new VelocityVoltage(0);
  private final VelocityVoltage lowerVelocity = new VelocityVoltage(0);

  double upperkP = 0.02, upperkI = 0, upperkD = 0.000190, upperkFF = 0.114,
         lowerkP = 0.02, lowerkI = 0, lowerkD = 0.000190, lowerkFF = 0.114;
  double upperSetPoint = 0, lowerSetPoint = 0;

  public Shooter() {
    upperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  //Chekc later motor direction
    lowerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    upperConfig.Slot0.kP = upperkP;
    upperConfig.Slot0.kI = upperkI;
    upperConfig.Slot0.kD = upperkD; 
    upperConfig.Slot0.kV = upperkFF; 
    lowerConfig.Slot0.kP = lowerkP; 
    lowerConfig.Slot0.kI = lowerkI;
    lowerConfig.Slot0.kD = lowerkD; 
    lowerConfig.Slot0.kV = lowerkFF;
  
    upperFlyWheel.getConfigurator().apply(upperConfig);
    lowerFlyWheel.getConfigurator().apply(lowerConfig);

    SmartDashboard.putNumber("Upper kP", upperkP);   SmartDashboard.putNumber("Lower kP", lowerkP);
    SmartDashboard.putNumber("Upper kI", upperkI);   SmartDashboard.putNumber("Lower kI", lowerkI);
    SmartDashboard.putNumber("Upper kD", upperkD);   SmartDashboard.putNumber("Lower kD", lowerkD);
    SmartDashboard.putNumber("Upper kFF", upperkFF);   SmartDashboard.putNumber("Lower kFF", lowerkFF);

    SmartDashboard.putNumber("Upper RPM", upperSetPoint);   SmartDashboard.putNumber("Lower RPM", lowerSetPoint);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     double ukP = SmartDashboard.getNumber("Upper kP", 0.02),
            ukI = SmartDashboard.getNumber("Upper kI", 0),
            ukD = SmartDashboard.getNumber("Upper kD", 0.000190), 
            lkP = SmartDashboard.getNumber("Lower kP", 0.02),
            lkI = SmartDashboard.getNumber("Lower kI", 0),
            lkD = SmartDashboard.getNumber("Lower kD", 0.000190),
            ukFF = SmartDashboard.getNumber("Upper kFF",0.114),
            lkFF = SmartDashboard.getNumber("Lower kFF",0.114); 

    if (upperkP != ukP){ upperkP = ukP; upperConfig.Slot0.kP = upperkP; upperFlyWheel.getConfigurator().apply(upperConfig);}
    if (upperkI != ukI){ upperkI = ukI; upperConfig.Slot0.kI = upperkI; upperFlyWheel.getConfigurator().apply(upperConfig);}
    if (upperkD != ukD){ upperkD = ukD; upperConfig.Slot0.kD = upperkD; upperFlyWheel.getConfigurator().apply(upperConfig);}
    if (upperkFF != ukFF){ upperkFF = ukFF; upperConfig.Slot0.kV = upperkFF; upperFlyWheel.getConfigurator().apply(upperConfig);}
    if (lowerkP != lkP){ lowerkP = lkP; lowerConfig.Slot0.kP = lowerkP; lowerFlyWheel.getConfigurator().apply(lowerConfig);}
    if (lowerkI != lkI){ lowerkI = lkI; lowerConfig.Slot0.kI = lowerkI; lowerFlyWheel.getConfigurator().apply(lowerConfig);}
    if (lowerkD != lkD){ lowerkD = lkD; lowerConfig.Slot0.kD = lowerkD; lowerFlyWheel.getConfigurator().apply(lowerConfig);}
    if (lowerkFF != lkFF){ lowerkFF = lkFF; lowerConfig.Slot0.kV = lowerkFF;
       lowerFlyWheel.getConfigurator().apply(lowerConfig);}

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
}
