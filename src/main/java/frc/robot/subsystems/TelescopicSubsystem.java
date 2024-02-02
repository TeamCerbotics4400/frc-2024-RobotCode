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
import frc.robot.Constants.ArmExtentionConstants;

public class TelescopicSubsystem extends SubsystemBase {

  private final CANSparkMax armExtendMotor = new CANSparkMax(ArmExtentionConstants.TELESCOPE_ID,MotorType.kBrushless);
  private RelativeEncoder telescopeEncoder = armExtendMotor.getEncoder();
  private SparkPIDController telescopicPID;
  private double actualArmExtensionPos;
  private double telescopicSetPoint = 0;
  private double telescopicSpeed = 0;

  public TelescopicSubsystem() {

    armExtendMotor.restoreFactoryDefaults();
    armExtendMotor.setInverted(true);
    armExtendMotor.setSmartCurrentLimit(80);
    armExtendMotor.setCANTimeout(0);
    armExtendMotor.setIdleMode(IdleMode.kBrake);

    telescopicPID = armExtendMotor.getPIDController();
    telescopeEncoder = armExtendMotor.getEncoder();

    telescopicPID.setP(ArmExtentionConstants.kP);
    telescopicPID.setI(ArmExtentionConstants.kI);
    telescopicPID.setD(ArmExtentionConstants.kD);
    telescopicPID.setFF(ArmExtentionConstants.kFF);

  }

  @Override
  public void periodic() {
     
      actualArmExtensionPos = (telescopeEncoder.getPosition()
      / ArmExtentionConstants.ARM_EXTENSION_GEAR_RATIO) / ArmExtentionConstants.NEO_COUNTS_PER_REV;

  SmartDashboard.putNumber("Actual Arm Extension Position: ", actualArmExtensionPos);

  double tRPM = SmartDashboard.getNumber("Telescopic RPM", 0);

  if (telescopicSetPoint != tRPM){telescopicSetPoint = tRPM;}

  }   
  public void setSoftLimits(){
    armExtendMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armExtendMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    armExtendMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 6);
    armExtendMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);   
  }
  public void TelescopicUp(){
    telescopicSpeed = telescopicSetPoint;
    telescopicPID.setReference(telescopicSpeed,ControlType.kVelocity);
  }
  public void telescsopicDown(){
    telescopicSpeed = -telescopicSetPoint;
    telescopicPID.setReference(telescopicSpeed,ControlType.kVelocity);
  }
}


