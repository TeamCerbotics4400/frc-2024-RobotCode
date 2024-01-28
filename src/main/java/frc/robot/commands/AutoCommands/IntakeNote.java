// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommands.IntakeCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNote extends ParallelCommandGroup {
  /** Creates a new IntakeCube. */
  IntakeSubsystem m_intake;
  ArmSubsystem m_arm;

  public IntakeNote(IntakeSubsystem m_intake, ArmSubsystem m_arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_intake = m_intake;
    this.m_arm = m_arm;

    addCommands(m_arm.goToPosition(ArmConstants.BACK_FLOOR_POSITION), 
    new IntakeCommand(m_intake));
  }
}
