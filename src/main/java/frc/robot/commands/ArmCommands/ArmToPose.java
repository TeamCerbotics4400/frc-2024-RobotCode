// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPose extends Command {
  /** Creates a new ArmToPose. */
  ArmSubsystem m_arm;
  DoubleSupplier m_angle;

  public ArmToPose(ArmSubsystem m_arm, DoubleSupplier m_angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_angle = m_angle;

    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_arm.isEnabled()){m_arm.enable();}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_angle.getAsDouble();
    
    m_arm.updateArmSetpoint(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.updateArmSetpoint(160.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
