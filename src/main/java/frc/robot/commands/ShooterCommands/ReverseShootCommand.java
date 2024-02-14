// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseShootCommand extends Command {
  /** Creates a new ReverseShootCommand. */
  ShooterSubsystem shooter;
  public ReverseShootCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setleftSpeed(-1000);
    shooter.setrightSpeed(-1000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopleft();
    shooter.stopright();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getVoltage() > 3.5;
  }
}
