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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
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

  private final Pigeon2 imu = new Pigeon2(DriveConstants.IMU_ID);

  private VisionSubsystem m_vision = new VisionSubsystem(this);  
  
  private SwerveDriveOdometry encoderOdometry = new SwerveDriveOdometry(DriveConstants.kSwerveKinematics, 
  getRotation2d(), getModulePositions(), new Pose2d());

  StructPublisher<Pose2d> encoderOdoPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Encoder Odometry", Pose2d.struct).publish();

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){}
    }).start();

    AutoBuilder.configureHolonomic(
      () -> m_vision.estimatedPose2d(),
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
        () -> {
          var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
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

    SmartDashboard.putNumber("Angular Acceleration", getAngularAcceleration());

    SmartDashboard.putNumber("Odometry X", m_vision.estimatedPose2d().getX());
    SmartDashboard.putNumber("Odometry Y", m_vision.estimatedPose2d().getY());
    SmartDashboard.putNumber("Odometry Rotation", 
                            m_vision.estimatedPose2d().getRotation().getDegrees());

    SmartDashboard.putNumber("IMU Angle", getHeading()); 

    SmartDashboard.putNumber("Odometry Angle Rads", Units.degreesToRadians(getHeading())); 

    SmartDashboard.putNumber("Distance to current target", m_vision.getDistanceToTarget());

    encoderOdometry.update(getRotation2d(), getModulePositions());

    encoderOdoPublisher.set(getEncoderOdometry());
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

  public double getAverageDriveSpeed(){
    double sumVelocities = 0.0;

    for(SwerveModule mod : swerveModules){
      sumVelocities += Math.abs(mod.getDriveVelocity());
    }

    return sumVelocities / swerveModules.length;
  }

  public double getAngularAcceleration(){
    return imu.getAngularVelocityZWorld().getValueAsDouble();
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
    m_vision.resetPoseEstimator(initPose);
  }

  public VisionSubsystem getVisionSubsystem(){
    return m_vision;
  }
  
  //Debug
  public void tuneDrivePID(double speedMtsPerSec){
    for(SwerveModule mod : swerveModules){
      mod.tuneModulePID(speedMtsPerSec);
    }
  }
  public Pose2d getEncoderOdometry(){
    return encoderOdometry.getPoseMeters();
  }
  public void resetEncoderOdometry(Pose2d initPose){
    encoderOdometry.resetPosition(getRotation2d(), getModulePositions(), initPose);
  }
}