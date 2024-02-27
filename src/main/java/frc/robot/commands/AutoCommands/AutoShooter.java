// Copylower (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShooter extends Command {

  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  ArmSubsystem arm;
  

  public AutoShooter(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;

    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.isInPosition()){
    shooter.setupperSpeed(4000);
    shooter.setlowerSpeed(4000);
      if (shooter.getRPM() > 3100){   
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
        if(!intake.noteInside()){
          return true;
        }
        else{
          return false;
        }
  
  }


}
