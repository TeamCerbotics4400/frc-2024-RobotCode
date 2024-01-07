// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveTrain;

public class TeleopControl extends Command {
  private final DriveTrain m_drive;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOriented;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  /** Creates a new TeleopControl. */
  public TeleopControl(DriveTrain m_drive, Supplier<Double> xSpdFunction, 
  Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, 
  Supplier<Boolean> fieldOriented) {

    this.m_drive = m_drive;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOriented = fieldOriented;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    // Use addRequirements() here to declare subsystem dependencies.
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

    xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > IOConstants.kDeadband ? turningSpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) 
          * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;

    if(fieldOriented.get()){
      //Relative to the field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, 
        ySpeed, 
        turningSpeed, 
        m_drive.getRotation2d());
        
    } else {
      //Relative to robot (NOT RECOMMENDED)
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    SwerveModuleState[] moduleStates = 
            DriveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    m_drive.setModuleStates(moduleStates, true);
    
    SmartDashboard.putBoolean("Is Field Oriented", fieldOriented.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
