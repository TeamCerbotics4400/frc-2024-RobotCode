// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class TelescopicSubsystem extends SubsystemBase {

  private final CANSparkMax armExtendMotor = 
                      new CANSparkMax(ArmConstants.TELESCOPE_ID,MotorType.kBrushless);
  private RelativeEncoder telescopeEncoder = armExtendMotor.getEncoder();
  private SparkPIDController telescopicPID;

  private double retractedTelescopic = 0.0;

  public TelescopicSubsystem() {

    armExtendMotor.restoreFactoryDefaults();

    armExtendMotor.setInverted(true);
    
    armExtendMotor.setSmartCurrentLimit(80);

    armExtendMotor.setCANTimeout(0);

    armExtendMotor.setIdleMode(IdleMode.kBrake);

    armExtendMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armExtendMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    telescopicPID = armExtendMotor.getPIDController();
    telescopeEncoder = armExtendMotor.getEncoder();

    telescopicPID.setP(ArmConstants.kTP);
    telescopicPID.setI(ArmConstants.kTI);
    telescopicPID.setD(ArmConstants.kTD);
    telescopicPID.setFF(ArmConstants.kTFF);

    telescopeEncoder.setPositionConversionFactor(ArmConstants.TELESCOPING_GEAR_RATIO);

    setSoftLimits();
  }

  @Override
  public void periodic() {
  SmartDashboard.putNumber("Telescoping Arm Position", getTelescopingPosition());
  }   

  public void setSoftLimits(){
    //Check Encoder Max Pos
    armExtendMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 6);
    armExtendMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);   
  }

  public void extendTelescopic(double meters){
    telescopicPID.setReference(meters, ControlType.kPosition);
  }

  public void retractTelescopic(){
    telescopicPID.setReference(retractedTelescopic, ControlType.kPosition);
  }

  public double getTelescopingPosition(){
    return telescopeEncoder.getPosition();
  }
}


