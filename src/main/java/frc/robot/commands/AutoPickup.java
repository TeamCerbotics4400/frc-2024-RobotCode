// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoPickup extends Command {
  /** Creates a new AutoPickup. */
  CommandSwerveDrivetrain m_drive;

  private final double xSpdFunction, ySpdFunction;

  private double pidOutput = 0.0;

  PIDController aimController;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(DriveConstants.MaxSpeed * 0.1)
  .withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutoPickup(CommandSwerveDrivetrain m_drive, double xSpdFunction
  ,  double ySpdFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;

    aimController = new PIDController(0.13, 0, 0.001);
    aimController.enableContinuousInput(-180, 180);
    aimController.setTolerance(1.0);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpdFunction;
    double ySpeed = ySpdFunction;

    pidOutput = -aimController.calculate(LimelightHelpers.getTX(VisionConstants.tagLimelightName));

    m_drive.applyRequest(
      () -> drive.withVelocityX(xSpeed)
      .withVelocityY(ySpeed)
      .withRotationalRate(pidOutput));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
