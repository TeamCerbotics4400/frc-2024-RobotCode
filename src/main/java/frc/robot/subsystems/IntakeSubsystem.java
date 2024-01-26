// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intake = new CANSparkMax(6,MotorType.kBrushless);
  public RelativeEncoder intakeEncoder = intake.getEncoder();

  public IntakeSubsystem() {
    intake.restoreFactoryDefaults();
    intake.setInverted(false);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Outake velocity", getIntakeVelocity());
        SmartDashboard.putNumber("Outake position", getIntakePosition());

  }

  public void startIntaking(){
    intake.set(-1);
  }//change order
    public void startOutaking(){
    intake.set(1);
  }
  public void stopIntaking(){
    intake.set(0.0);
  }
  public double getIntakeVelocity(){
    return intakeEncoder.getVelocity();
  }
  public double getIntakePosition(){
    return intakeEncoder.getPosition();
  }
  public void resetEncoder(){
    intakeEncoder.setPosition(0);
  }
}