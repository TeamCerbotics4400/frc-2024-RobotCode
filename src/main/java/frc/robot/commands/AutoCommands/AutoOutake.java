// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoOutake extends Command {
IntakeSubsystem intake;
Timer timer = new Timer();

  public AutoOutake(IntakeSubsystem intake) {

this.intake = intake;
addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.noteInside()){
      timer.start();
      intake.smallOutake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntaking();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > 0.1){
      timer.stop();
      timer.reset();
      return true;
    }
    return false;
  }
}
