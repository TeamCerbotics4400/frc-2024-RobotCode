// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

  ShooterSubsystem shooter;
  IntakeSubsystem intake;

  public ShooterCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {

    this.shooter = shooter;
    this.intake = intake;
      
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setleftSpeed();
    shooter.setrightSpeed();
     
    if (shooter.getRPM() > 5000){   //check to change rpm
      intake.startIntaking();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopleft();
    shooter.stopright();
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
