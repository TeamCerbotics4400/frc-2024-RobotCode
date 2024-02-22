// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAMPCommand extends Command {
  /** Creates a new ShooterAMPCommand. */
  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  public ShooterAMPCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
this.shooter = shooter;
this.intake = intake;

addRequirements(shooter, intake);  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setleftSpeed(3000);
    shooter.setrightSpeed(3000);
    intake.startIntaking();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopleft();
    shooter.stopright();
    intake.stopIntaking();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
