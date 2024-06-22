// Copylower (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSafeFail extends Command {

  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  ArmSubsystem arm;
  Timer timer = new Timer();
  double test = timer.get();

  
  public ShooterSafeFail(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    test = timer.get();
          if (arm.isInPosition()){
              if (shooter.getRPM() >= 3000){   
             intake.startIntaking();
          }
        }
     if (test >= 1.75){  
      intake.startIntaking();
      timer.reset();
      timer.stop();
      }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopupper();
    shooter.stoplower();
    intake.stopIntaking();
    timer.reset();
    timer.stop();
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