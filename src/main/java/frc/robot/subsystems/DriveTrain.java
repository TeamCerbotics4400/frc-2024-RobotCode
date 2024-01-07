// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  public SwerveModule[] swerveModules = new SwerveModule[]{
    //Left Front Module
    new SwerveModule(0, DriveConstants.Module0.CONSTANTS),
    //Right Front Module
    new SwerveModule(1, DriveConstants.Module1.CONSTANTS),
    //Back Right Module
    new SwerveModule(2, DriveConstants.Module2.CONSTANTS),
    //Back Left Module
    new SwerveModule(3, DriveConstants.Module3.CONSTANTS)
  };

  private final Pigeon2 imu = new Pigeon2(DriveConstants.IMU_ID, "rio");

  private VisionSubsystem m_vision = new VisionSubsystem(this);  

  //Project configured to use Encoder Odometry
  private SwerveDriveOdometry encoderOdo = 
  new SwerveDriveOdometry(
    DriveConstants.kSwerveKinematics, 
    getRotation2d(), 
    getModulePositions());

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){}
    }).start();

    AutoBuilder.configureHolonomic(
      () -> encoderOdo.getPoseMeters(),
      this::resetOdometryPose, 
      () -> getRobotRelativeSpeeds(), 
      this::setRobotRelativeSpeeds, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(
          DriveConstants.traslationP, 
          DriveConstants.traslationD), 
        new PIDConstants(
          DriveConstants.rotationP, 
          DriveConstants.rotationD), 
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond, 
        DriveConstants.kDriveBaseRadius, 
        new ReplanningConfig()), 
      this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    for(SwerveModule mod : swerveModules){
      SmartDashboard.putNumber("Module [" + mod.moduleNumber + "] Absolute Encoder", 
      swerveModules[mod.moduleNumber].getAngleDeegrees());
    }

    for(SwerveModule mod: swerveModules){
      SmartDashboard.putNumber("Module [" + mod.moduleNumber + "] Velocity",
       swerveModules[mod.moduleNumber].getDriveVelocity());
    }

    encoderOdo.update(getRotation2d(), getModulePositions());

    SmartDashboard.putNumber("Odometry X", m_vision.estimatedPose2d().getX());
    SmartDashboard.putNumber("Odometry Y", m_vision.estimatedPose2d().getY());
    SmartDashboard.putNumber("Odometry Rotation", 
                            m_vision.estimatedPose2d().getRotation().getDegrees());


    SmartDashboard.putNumber("IMU Angle", getHeading());  
  
  }

  public void zeroHeading(){
    imu.setYaw(0);
  }

  public double getHeading(){
    return Math.IEEEremainder(imu.getYaw().getValueAsDouble(), 360);
  }

  public void resetModuleEncoders(){
    for(SwerveModule mod : swerveModules){
      mod.resetEncoders();
    }
  }

  public void stopModules(){
    for(SwerveModule mod : swerveModules){
      mod.stop();
    }
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop){
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, 
      DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    for(SwerveModule mod : swerveModules){
      mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule mod : swerveModules){
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : swerveModules){
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void lockWheels(){
    for(SwerveModule mod : swerveModules){
      mod.lockModule();
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speeds){
    SwerveModuleState[] moduleStates = 
    DriveConstants.kSwerveKinematics.toSwerveModuleStates(speeds);

    setModuleStates(moduleStates, false);
  }

  public void setAllianceForVision(Alliance alliance){
    m_vision.setAlliance(alliance);
  }
  
  public void resetOdometryPose(Pose2d initPose){
    encoderOdo.resetPosition(getRotation2d(), getModulePositions(), initPose);
    //m_vision.resetPoseEstimator(pose);
  }

  //Debug
  public void tuneDrivePID(double speedMtsPerSec){
    for(SwerveModule mod : swerveModules){
      mod.tuneModulePID(speedMtsPerSec);
    }
  }
}