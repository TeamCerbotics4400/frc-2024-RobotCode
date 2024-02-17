// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class WaitForSensorChange extends Command {
  /** Creates a new WaitForSensorChange. */
  final boolean goalState;
  final IntakeSubsystem m_intake;
  boolean detected;

  public WaitForSensorChange(IntakeSubsystem m_intake, boolean goalState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.goalState = goalState;
    this.m_intake = m_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    detected = m_intake.noteInside();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return detected == goalState;
  }
}
