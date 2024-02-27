// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OutakeCommand extends Command {
  /** Creates a new OutakeCommand. */
  IntakeSubsystem intake;
  ShooterSubsystem shooter;

  public OutakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shooter = shooter;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.smallOutake();
    shooter.setlowerSpeed(-500);
    shooter.setupperSpeed(-900);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntaking();
    shooter.stoplower();
    shooter.stopupper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    if (intake.getIntakePosition() > -0.3){   // 40 rpm
      intake.resetEncoder();
      intake.stopIntaking();

      return true;
    }
     Roman's method*/
    return false;
    
  }
}
