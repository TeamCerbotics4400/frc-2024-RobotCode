// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CookShooter extends Command {
  /** Creates a new CookShooter. */
  ShooterSubsystem m_shooter;
  LEDSubsystem m_leds;
  public CookShooter(ShooterSubsystem m_shooter, LEDSubsystem m_leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;
    this.m_leds = m_leds;

    addRequirements(m_shooter, m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setupperSpeed(2400);
    m_shooter.setlowerSpeed(  2400);
    
    if(m_shooter.getRPM() > 2700){
      m_leds.strobeColor1();
    } else {
      m_leds.strobeBlue();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stoplower();
    m_shooter.stopupper();
    m_leds.setBlue();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
