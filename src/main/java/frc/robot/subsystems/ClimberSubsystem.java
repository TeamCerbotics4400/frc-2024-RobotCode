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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax climberMotor = 
                new CANSparkMax(ClimberConstants.CLIMBER_ID, MotorType.kBrushless);
  private final SparkPIDController climberPIDController;
  private final RelativeEncoder climberEncoder;

  double leftkP = 0.0, leftkI = 0.0, leftkD = 0.0, leftkFF = 0.0;

  private double openExtendedClimber = 1.0;  

  private SendableChooser<String> climberChooser = new SendableChooser<>();
  private String climberMOdeSelector;
  private final String[] modeNames = {"BRAKE", "COAST"};

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

    climberEncoder.setPositionConversionFactor(ClimberConstants.CLIMBER_GEARBOX);

    resetEncoder();

    climberChooser.setDefaultOption("Brake Mode", modeNames[0]);
    climberChooser.addOption("Brake Mode", modeNames[0]);
    climberChooser.addOption("Coast Mode", modeNames[1]);

    SmartDashboard.putData("Climber Mode", climberChooser);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position",getClimberPosition()); 

          if(DriverStation.isDisabled()){
        climberMOdeSelector = climberChooser.getSelected();
        switch (climberMOdeSelector) {
          case "BRAKE":
            setBrake();
          break;

          case "COAST":
            setCoast();
          break;
        }
      } else {
        setBrake();
      }
  }
  
  public double getClimberPosition(){
    return climberEncoder.getPosition();
  }


    //Redo climber values
  public void extendClimber(){
    climberPIDController.setReference(2100.0, ControlType.kPosition);
  }

  public void retractClimber(){
    climberPIDController.setReference(3852.0, ControlType.kPosition);
  }
//



  public void openLoopClimber(){
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

  public void setBrake(){
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast(){
    climberMotor.setIdleMode(IdleMode.kCoast);
  }
}
 