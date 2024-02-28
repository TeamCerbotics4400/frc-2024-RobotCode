// Copylower (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.POISelector;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  ArmSubsystem arm;
  POISelector selector;
  

  public ShooterCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm, POISelector selector) {

    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;
    this.selector = selector;

    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(selector.getShooterName()){
      case "AMP":
      shooter.setupperSpeed(2000);
      shooter.setlowerSpeed(500);
      intake.startIntaking();
      break;
      case "Speaker":
          if (arm.isInPosition()){
       shooter.setupperSpeed(4000);
       shooter.setlowerSpeed(4000);
          if (shooter.getRPM() >= 3200){   
             intake.startIntaking();
          }
        }
      break;
      case "Trap":
          if (arm.isInPosition()){
        shooter.setupperSpeed(3300);
        shooter.setlowerSpeed(3300);
          if (shooter.getRPM() >= 2500){   
          intake.startIntaking();
          } 
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
