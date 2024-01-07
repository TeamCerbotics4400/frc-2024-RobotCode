// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class VisionSubsystem {
    private final DriveTrain m_drive;

    private final SwerveDrivePoseEstimator m_poseEstimator;

    Alliance alliance = Alliance.Blue;

    Field2d m_field = new Field2d();

    Debouncer poseDebouncer = new Debouncer(0.1, DebounceType.kRising);

    public VisionSubsystem(DriveTrain m_drive){
        this.m_drive = m_drive;

        m_poseEstimator = 
            new SwerveDrivePoseEstimator(DriveConstants.kSwerveKinematics,
            m_drive.getRotation2d(), m_drive.getModulePositions(), 
            new Pose2d(0,0 , m_drive.getRotation2d()));

        SmartDashboard.putData("Field", m_field);

        vision_thread();
    }

    //Start the vision systeam in a different CPU thread for better command scheduler performance
    public void vision_thread(){
        try{
            new Thread(() -> {
                while(true){
                    periodic();
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException e){
                     //Auto-generated catch block
                     e.printStackTrace();
                    }
                }
            }).start();
        } catch(Exception e){}
    }

    public void periodic(){
        odometryWvision();
        areTagsatGoodRange();
        setDynamicVisionStdDevs();

        SmartDashboard.putString("Alliance", alliance.toString());

        SmartDashboard.putBoolean("Tags good range", poseDebouncer.calculate(areTagsatGoodRange()));

        SmartDashboard.putNumber("Num of tags", getNumofDetectedTargets());
    }

    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }

    public void resetPoseEstimator(Pose2d pose){
        m_poseEstimator.resetPosition(m_drive.getRotation2d(), m_drive.getModulePositions(), pose);
      }

    public Pose2d estimatedPose2d(){
        return m_poseEstimator.getEstimatedPosition();
    }

    public Translation2d getEstimationTranslation(){
        return m_poseEstimator.getEstimatedPosition().getTranslation();
    }
    
    public Rotation2d getEstimationRotation(){
        return m_poseEstimator.getEstimatedPosition().getRotation();
    }
    
    public double getEstimationAngle(){
        return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    /* 
   * On board we have two cameras, a Limelight 2+ for use on retroreflective targets and a 
   * Limelight 3. The relevant camera in this method is the Limelight 3, 
   * where we are using it for fudicial targets tracking.
   * 
   * In this method we call our co-processor and camera, get what it detects and send that data
   * to the pose estimator to correct our odometry. If no apriltag is detected, the robot will
   * continue using the motor encoders and the mounted gyro to change it's position on the field.
   * 
   * All of this data is then sent to a Fied2d() widget on the Shuffleboard and logged
   * for later visualization.
   * 
   * After 2 regionals and 1 week before Worlds, we also added a rejection conditional. If the 
   * detected tag is at or over a certain distance we just dont use that data. 
   * It gives a much cleaner and less noisy estimation.
  */
  public void odometryWvision(){
    m_poseEstimator.update(m_drive.getRotation2d(), m_drive.getModulePositions());

    LimelightHelpers.Results results = 
        LimelightHelpers.getLatestResults(VisionConstants.tagLimelightName).targetingResults;

    if(LimelightHelpers.getTV(VisionConstants.tagLimelightName) && poseDebouncer.calculate(areTagsatGoodRange())){
      Pose2d camPose = LimelightHelpers.toPose2D(results.botpose_wpiblue);
      m_poseEstimator.addVisionMeasurement(camPose, 
      Timer.getFPGATimestamp() - (results.latency_capture / 1000.0)
       - (results.latency_pipeline / 1000.0));
      m_field.getObject("Cam est Pose").setPose(camPose);
    } else {
      m_field.getObject("Cam est Pose").setPose(m_poseEstimator.getEstimatedPosition());
    }

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  public void dynamicVisionDvs(){
    double xyStds = 0;
    double degStds = 0;
    if(getNumofDetectedTargets() >= 2 && poseDebouncer.calculate(areTagsatGoodRange())){
      xyStds = 0.1;
      degStds = 0.1;
    } else if(LimelightHelpers.getTA(VisionConstants.tagLimelightName) >= 0.5 && poseDebouncer.calculate(areTagsatGoodRange())){
      xyStds = 0.5;
      degStds = 0.5;
    } else if(LimelightHelpers.getTA(VisionConstants.tagLimelightName) >= 0.1 && poseDebouncer.calculate(areTagsatGoodRange())){
      xyStds = 1.2;
      degStds = 1.2;
    }

    m_poseEstimator.setVisionMeasurementStdDevs(
                  VecBuilder.fill(xyStds, xyStds, degStds));
  }

  //The rejection method in question
  public boolean areTagsatGoodRange(){
    boolean goodRange = false;
    //If Alliance is Blue, use Blue Community
    if(alliance == Alliance.Blue){
      if(getNumofDetectedTargets() <= 1){
        if(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.tagLimelightName).getX() <= 3.5){
          goodRange = true;
        } else {
          goodRange = false;
        }
      } else {
        if(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.tagLimelightName).getX() <= 6.0){
          goodRange = true;
        } else {
          goodRange = false;
        }
      }
    //If Alliance is Red, use Red Community
    } else {
      if(getNumofDetectedTargets() <= 1){
        if(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.tagLimelightName).getX() >= 13.0){
          goodRange = true;
        } else {
          goodRange = false;
        }
      } else {
        if(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.tagLimelightName).getX() >= 10.50){
          goodRange = true;
        } else {
          goodRange = false;
        }
      }
    } 
    
    return goodRange;
  }

  public void setDynamicVisionStdDevs(){
    if(getNumofDetectedTargets() <= 2){
      m_poseEstimator.setVisionMeasurementStdDevs(new 
                                MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1));
    } else {
      if(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.tagLimelightName).getX() <= 2.5){
        m_poseEstimator.setVisionMeasurementStdDevs(new 
                                MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1));
      } else if(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.tagLimelightName).getX() >= 2.5 
          && LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.tagLimelightName).getX() <= 3.5){
            m_poseEstimator.setVisionMeasurementStdDevs(new 
            MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.3, 0.3, 0.3));
      } else if(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.tagLimelightName).getX() >= 2.5 
      && LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.tagLimelightName).getX() <= 3.5){
        m_poseEstimator.setVisionMeasurementStdDevs(new 
            MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.7, 0.7, 0.7));
      } else {
        m_poseEstimator.setVisionMeasurementStdDevs(new 
            MatBuilder<>(Nat.N3(), Nat.N1()).fill(1.0, 1.0, 1.0));
      }
    }
  }

  /*public void positionState(){
    if(DriverStation.isTeleop()){
      if(m_poseEstimator.getEstimatedPosition().getY() <= 1.45){
        StateMachines.setPositionState(PositionState.CABLE);
      } else if(m_poseEstimator.getEstimatedPosition().getY() >= 4.0){
        StateMachines.setPositionState(PositionState.LOADING);
      } else {
        StateMachines.setPositionState(PositionState.MIDDLE);
      }
    } else {
      StateMachines.setPositionState(PositionState.TELE);
    }
  }*/

  public int getNumofDetectedTargets(){
    return LimelightHelpers
    .getLatestResults(VisionConstants.tagLimelightName).targetingResults.targets_Fiducials.length;
  }
}