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

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  
  CANSparkMax upperFlyWheel = new CANSparkMax (1,MotorType.kBrushless);
  CANSparkMax lowerFlyWheel = new CANSparkMax (6,MotorType.kBrushless);

  RelativeEncoder lowerEncoder;
  RelativeEncoder upperEncoder;

  SparkPIDController upperController; 
  SparkPIDController lowerController; 

  private double upperVelocity = 0;
  private double lowerVelocity = 0;

  double upperkP = 0.02, upperkI = 0, upperkD = 0.000190, upperkFF = 0.114,
         lowerkP = 0.02, lowerkI = 0, lowerkD = 0.000190, lowerkFF = 0.114;
  double upperSetPoint = 0, lowerSetPoint = 0;

  public Shooter() {
    upperFlyWheel.restoreFactoryDefaults();
    lowerFlyWheel.restoreFactoryDefaults();

    lowerFlyWheel.setInverted(true);
    upperFlyWheel.setInverted(false);

    lowerFlyWheel.setIdleMode(IdleMode.kBrake);
    upperFlyWheel.setIdleMode(IdleMode.kBrake);

    lowerFlyWheel.clearFaults();
    upperFlyWheel.clearFaults();

    upperController = upperFlyWheel.getPIDController();
    lowerController = lowerFlyWheel.getPIDController();

    upperEncoder = upperFlyWheel.getEncoder();
    lowerEncoder = lowerFlyWheel.getEncoder();

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

    if (upperkP != ukP){ upperkP = ukP; upperController.setP(upperkP); }
    if (upperkI != ukI){ upperkI = ukI; upperController.setI(upperkI); }
    if (upperkD != ukD){ upperkD = ukD; upperController.setD(upperkD); }
    if (upperkFF != ukFF){ upperkFF = ukFF; upperController.setFF(upperkFF);}
    if (lowerkP != lkP){ lowerkP = lkP; lowerController.setP(lowerkP); }
    if (lowerkI != lkI){ lowerkI = lkI; lowerController.setI(lowerkI); }
    if (lowerkD != lkD){ lowerkD = lkD; lowerController.setD(lowerkD);}
    if (lowerkFF != lkFF){ lowerkFF = lkFF; lowerController.setFF(lowerkFF);}

      double uRPM = SmartDashboard.getNumber("Upper RPM", 0),
             lRPM = SmartDashboard.getNumber("Lower RPM", 0);

    if (upperSetPoint != uRPM){upperSetPoint = uRPM;}
    if (lowerSetPoint != lRPM){lowerSetPoint = lRPM;}

    SmartDashboard.putNumber("Current Upper RPM", upperEncoder.getVelocity());
    SmartDashboard.putNumber("Current Lower RPM", lowerEncoder.getVelocity());

  }


  public void setUpperSpeed(){
    upperVelocity = upperSetPoint;
    upperController.setReference(upperVelocity,ControlType.kVelocity);
  }
  public void setLowerSpeed(){
    lowerVelocity = lowerSetPoint;
    lowerController.setReference(lowerVelocity,ControlType.kVelocity);
  }
    public void stopUpper(){
    upperFlyWheel.set(0);
  }

  public void stopLower(){
    lowerFlyWheel.set(0);
  }
}
