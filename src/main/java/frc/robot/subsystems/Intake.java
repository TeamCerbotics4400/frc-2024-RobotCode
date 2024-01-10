// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intake = new CANSparkMax(0,MotorType.kBrushless);
  public Intake() {
    intake.restoreFactoryDefaults();
    intake.setInverted(false);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startIntaking(){
    intake.set(1.0);
  }
    public void startOutaking(){
    intake.set(-1.0);
  }
}
