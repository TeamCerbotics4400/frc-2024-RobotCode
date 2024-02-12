// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ShooterCommands.ShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmShoot extends ParallelCommandGroup {
  /** Creates a new ArmShoot. */
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  ArmSubsystem m_arm;

  public ArmShoot(ShooterSubsystem m_shooter, IntakeSubsystem m_intake, ArmSubsystem m_arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_arm = m_arm;
    this.m_intake = m_intake;
    this.m_shooter = m_shooter;
    addCommands(m_arm.goToPosition(160), new ShooterCommand(m_shooter, m_intake, m_arm));
  }
}
