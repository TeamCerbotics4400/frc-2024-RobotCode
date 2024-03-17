// Copyright (c) FIRST and other WPILib contributors.
// Copylower (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LastShoot extends Command {

  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  ArmSubsystem arm;
  
  public LastShoot(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //shooter.setupperSpeed(3200);  //Shooter now starts instead of waiting for the arm
     //shooter.setlowerSpeed(3200);
          if (arm.isInPosition()){
              if (shooter.getRPM() >= 2200){   
             intake.startIntaking();
          }
        }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopupper();
    shooter.stoplower();
    intake.stopIntaking();
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
