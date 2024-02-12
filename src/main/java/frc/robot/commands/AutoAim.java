// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoAim extends Command {
  /** Creates a new AutoAim. */
  private final DriveTrain m_drive;

  private double pidOutput = 0.0;

  PIDController aimController;

  public AutoAim(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;

    aimController = new PIDController(0.13, 0, 0.001);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimController.reset();

    aimController.setTolerance(2.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidOutput = -aimController.calculate(LimelightHelpers.getTX(VisionConstants.tagLimelightName));

    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        0, 
        0, 
        pidOutput, 
        m_drive.getRotation2d());

    SwerveModuleState[] moduleStates = 
            DriveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    m_drive.setModuleStates(moduleStates, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return aimController.atSetpoint();
  }
}
