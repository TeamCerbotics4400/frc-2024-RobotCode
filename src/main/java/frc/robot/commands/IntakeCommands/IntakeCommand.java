// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  IntakeSubsystem intake;
  Timer timer = new Timer();

  public IntakeCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!intake.noteInside()){
      intake.startIntaking();
      intake.resetEncoder();
    }
    else{
      timer.start();
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntaking();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return timer.get() > .0;
  }
}