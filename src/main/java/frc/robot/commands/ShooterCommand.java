// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  ShooterSubsystem shooter;
//ArmSubsystem m_arm;


  public ShooterCommand(ShooterSubsystem shooter, ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    //this.m_arm = m_arm;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setUpperSpeed();
    shooter.setLowerSpeed();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopUpper();
    shooter.stopLower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if(DriverStation.isAutonomous()){
      //Timer.delay(0.5);
     /*  if(m_arm.isInShootingPos() && onRevs() && StateMachines.isShooting()){
        return true;
      } else {
        return false;
      }*/
    } 
    return false;
  }
}
