// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class OutakeCommand extends Command {
  /** Creates a new OutakeCommand. */
  Intake intake;
  public OutakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.startOutaking();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntaking();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   /* if (intake.getOutakeVelocity() <= 3500){
      return true;
    }
    else{*/
    return false;
    
  }
}
