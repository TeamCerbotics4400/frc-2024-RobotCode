// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignOdometry extends Command {
  /** Creates a new AutoAim. */
  enum Mode {
    NORMAL_CONTROL,
    ALIGN_TO_POINT
  }

  double theta;

  private Mode currentMode = Mode.NORMAL_CONTROL;

  private Translation2d targetPosition = new Translation2d(0.33, 5.45);
  
  private Pose2d robotPose;

  private final CommandSwerveDrivetrain m_drive;

  private final SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();

  private final PhoenixPIDController m_aimController = new PhoenixPIDController(1, 0.0, 0.0001);

  private double pidOutput = 0.0;

  public AutoAlignOdometry(CommandSwerveDrivetrain m_drive) {
    // Use addRequirements() here to decla re subsystem dependencies.
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
    robotPose = LimelightHelpers.getBotPose2d("limelight-tags");
    currentMode = Mode.ALIGN_TO_POINT;
  }

  // Called every time the scheduler Wruns while the command is scheduled.
  @Override
  public void execute() {

    theta = targetPosition.getAngle().getDegrees();

    pidOutput = 
        m_aimController.calculate(robotPose.getRotation().getDegrees(), 90, Timer.getFPGATimestamp());

    m_drive.setControl(drive.withSpeeds(new ChassisSpeeds(0, 0, pidOutput)));

    robotPose = LimelightHelpers.getBotPose2d("limelight-tags");



    
    System.out.println("Angulo de robot: " + robotPose.getRotation().getDegrees());
    System.out.println("Angulo deseado : " + theta);
    System.out.println("Error de angulo: " + (robotPose.getRotation().getDegrees() - theta));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentMode = Mode.NORMAL_CONTROL;
    m_drive.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return theta < 1;//m_aimController.atSetpoint();
  }
}
