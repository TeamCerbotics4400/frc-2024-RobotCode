// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RomanShoot extends Command {
  /** Creates a new RomanShoot. */
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;

  public RomanShoot(ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intake = m_intake;
    this.m_shooter = m_shooter;

    addRequirements(m_shooter, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setleftSpeed(6000);
    m_shooter.setrightSpeed(6000);
         
    if (m_shooter.getRPM() > 5000){   
      m_intake.startIntaking();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopleft();
    m_shooter.stopright();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
