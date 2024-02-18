// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DebugCommands.WaitForSensorChange;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeSequence extends Command {
  /** Creates a new TestSequence. */
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  public IntakeSequence(IntakeSubsystem m_intake, ShooterSubsystem m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intake = m_intake;
    this.m_shooter = m_shooter;

    addRequirements(m_intake, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.slowIn();
    m_shooter.setShooterSpeed(500);
    /*new ParallelCommandGroup(
      new InstantCommand(() -> m_intake.slowIn()),
      new InstantCommand(() -> m_shooter.setShooterSpeed(500)),
      new WaitForSensorChange(m_intake, true).withTimeout(10),
      new InstantCommand(() -> m_intake.stopIntaking()),
      new InstantCommand(() -> m_shooter.stopShooter()),

      new WaitCommand(0.5),
      new InstantCommand(() -> m_intake.slowOut()),
      new WaitForSensorChange(m_intake, false).withTimeout(1.0),
      new InstantCommand(() -> m_intake.stopIntaking())
    );*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntaking();
    m_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.noteInside();
  }

  public boolean hasIntaked(){
    if(m_intake.getIntakeVelocity() <= 0.10 && !m_intake.noteInside()){
      return true;
    } else {
      return false;
    }
  }
}