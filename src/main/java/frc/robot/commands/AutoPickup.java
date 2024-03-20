// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoPickup extends Command {
  /** Creates a new AutoPickup. */
  CommandSwerveDrivetrain m_drive;

  private double pidOutput = 0.0;

  private final PhoenixPIDController m_aimController = new PhoenixPIDController(0.08, 0.0, 0.0);

  private final SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();

  public AutoPickup(CommandSwerveDrivetrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;

    //m_aimController.setPID(0.2, 0.0, 0.0080);
    m_aimController.enableContinuousInput(-Math.PI, Math.PI);
    m_aimController.setTolerance(0.01);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_aimController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  pidOutput = 
        m_aimController.calculate(LimelightHelpers.getTX(VisionConstants.neuralLimelight), 0.0, Timer.getFPGATimestamp());

      m_drive.setControl(drive.withSpeeds(new ChassisSpeeds(0, 0, pidOutput)));

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
