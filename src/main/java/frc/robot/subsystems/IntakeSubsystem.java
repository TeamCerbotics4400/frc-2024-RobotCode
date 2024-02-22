// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_ID,MotorType.kBrushless);
  SparkPIDController intakeController = intakeMotor.getPIDController();

  public static DigitalInput intakeSensor = new DigitalInput(1); //Check what channel the sensor will be  on

  RelativeEncoder intakeEncoder;


  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.setInverted(true);

    intakeMotor.setSmartCurrentLimit(80);

    intakeEncoder = intakeMotor.getEncoder();

   }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note inside robot", noteInside());

    SmartDashboard.putBoolean("Intake Controller Reset", hasControllerReset());
  
  }

  public void startIntaking(){
    intakeMotor.set(-0.75);
  }

  public void startOutaking(){
    intakeMotor.set(1);
  }

  public void stopIntaking(){
    intakeMotor.set(0.0);
  }

  public static boolean noteInside(){
    return !intakeSensor.get();
  }

  public boolean hasControllerReset(){
    return intakeMotor.getStickyFault(FaultID.kHasReset);
  }
}