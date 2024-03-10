// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class CookShooter extends Command {
  /** Creates a new CookShooter. */
  ShooterSubsystem m_shooter;
  public CookShooter(ShooterSubsystem m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setupperSpeed(3200);
    m_shooter.setlowerSpeed(3200);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stoplower();
    m_shooter.stopupper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
