// Copylower (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShootCommand extends Command {

  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  ArmSubsystem arm;
  LEDSubsystem m_leds;
  
  public AmpShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm, LEDSubsystem m_leds) {

    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;
    this.m_leds = m_leds;

    addRequirements(shooter, intake, m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
       shooter.setupperSpeed(2000);
       shooter.setlowerSpeed(500);
       intake.startIntaking();
       m_leds.strobeBlue();

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopupper();
    shooter.stoplower();
    intake.stopIntaking();
    m_leds.setBlue();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if(DriverStation.isAutonomous()){
        if(!intake.noteInside()){
          return true;
        }
        return false;
    } 
    return false;
  }


}
