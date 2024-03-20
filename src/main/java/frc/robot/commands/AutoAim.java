// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends Command {
  /** Creates a new AutoAim. */
  private final CommandSwerveDrivetrain m_drive;
  private final VisionSubsystem m_vision;

  private final SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();

  private final PhoenixPIDController m_aimController = new PhoenixPIDController(0.4, 0.0, 0.0);

  private double pidOutput = 0.0;

  public AutoAim(CommandSwerveDrivetrain m_drive, VisionSubsystem m_vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.m_vision = m_vision;

    //m_aimController.setPID(0.2, 0.0, 0.0080);
    m_aimController.enableContinuousInput(-Math.PI, Math.PI);
    m_aimController.setTolerance(0.01);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_aimController.reset();
    m_vision.setCameraPipeline(VisionConstants.autoAim_Pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    pidOutput = 
        m_aimController.calculate(LimelightHelpers.getTX(VisionConstants.tagLimelight), 0.0, Timer.getFPGATimestamp());

    m_drive.setControl(drive.withSpeeds(new ChassisSpeeds(0, 0, pidOutput)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vision.setCameraPipeline(VisionConstants.main_Pipeline);
    m_drive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//m_aimController.atSetpoint();
  }
}
