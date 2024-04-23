// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class SmallerIntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  IntakeSubsystem intake;
  ShooterSubsystem shooter;


  public SmallerIntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;

    
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      intake.smallIntake(); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntaking();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intake.noteInside()){
      return true;
    } else {
      return false;
    }
  }
}