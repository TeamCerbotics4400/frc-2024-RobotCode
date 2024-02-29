// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_ID,MotorType.kBrushless);
  public  DigitalInput intakeSensor = new DigitalInput(2);

  RelativeEncoder intakeEncoder;

  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.setInverted(true);

    intakeMotor.setSmartCurrentLimit(80);
   }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note inside robot", noteInside());

    SmartDashboard.putBoolean("Intake Controller Reset", hasControllerReset());
  }

  public void startIntaking(){
    intakeMotor.set(-0.9);
  }

  public void startOutaking(){
    intakeMotor.set(0.5);
  }

  public void smallOutake(){
    intakeMotor.set(0.20);
  }

  public void stopIntaking(){
    intakeMotor.set(0.0);
  }

  public  boolean noteInside(){
    return !intakeSensor.get();
  }

  public boolean hasControllerReset(){
    return intakeMotor.getStickyFault(FaultID.kHasReset);
  }
}