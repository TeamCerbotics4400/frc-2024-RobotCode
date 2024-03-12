// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FieldCentricDrive extends Command {

  private final CommandSwerveDrivetrain m_drive;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DriveConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  /** Creates a new TeleOpControl. */
  public FieldCentricDrive(CommandSwerveDrivetrain m_drive, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
  Supplier<Double> turningSpdFunction) {

    this.m_drive = m_drive;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;  

    this.xLimiter = new SlewRateLimiter(DriveConstants.MaxSpeed);
    this.yLimiter = new SlewRateLimiter(DriveConstants.MaxSpeed);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.MaxAngularRate);
    
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.MaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.MaxSpeed;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.MaxAngularRate;

    m_drive.setControl(drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(turningSpeed));
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
