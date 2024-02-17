// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DebugCommands.WaitForSensorChange;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSequence extends SequentialCommandGroup {
  /** Creates a new IntakeSequence. */
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  public IntakeSequence(IntakeSubsystem m_intake, ShooterSubsystem m_shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_intake = m_intake;
    this.m_shooter = m_shooter;
    
    addRequirements(m_intake, m_shooter);

    addCommands(
      new InstantCommand(() -> m_intake.startIntaking()),
      new InstantCommand(() -> m_shooter.setShooterSpeed(300)),
      new WaitForSensorChange(m_intake, true).withTimeout(10),
      new InstantCommand(() -> m_intake.stopIntaking()),
      new InstantCommand(() -> m_shooter.stopShooter()),

      new WaitCommand(0.5),
      new InstantCommand(() -> m_intake.slowOut()),
      new WaitForSensorChange(m_intake, false).withTimeout(1.0),
      new InstantCommand(() -> m_intake.stopIntaking())
    );
  }
}
