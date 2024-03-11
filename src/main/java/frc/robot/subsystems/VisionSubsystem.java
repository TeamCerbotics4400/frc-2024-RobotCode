// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class VisionSubsystem {
    private final CommandSwerveDrivetrain m_drive;

    Alliance alliance = Alliance.Blue;

    Field2d m_field = new Field2d();

    Debouncer poseDebouncer = new Debouncer(0.1, DebounceType.kRising);

    public VisionSubsystem(CommandSwerveDrivetrain m_drive){
        this.m_drive = m_drive;

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
      if(DriverStation.isTeleop() && DriverStation.isEnabled()){
        odometryWvision();
      }
        
        setDynamicVisionStdDevs();

        SmartDashboard.putString("Alliance", alliance.toString());

        SmartDashboard.putNumber("Num of tags", getNumofDetectedTargets());
    }

    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }

    public void resetPoseEstimator(Pose2d pose){
        m_drive.seedFieldRelative(pose);
      }

    public Pose2d estimatedPose2d(){
        return m_drive.getState().Pose;
    }

    public Translation2d getEstimationTranslation(){
        return m_drive.getState().Pose.getTranslation();
    }
    
    public Rotation2d getEstimationRotation(){
        return m_drive.getState().Pose.getRotation();
    }
    
    public double getEstimationRotationDegrees(){
        return m_drive.getState().Pose.getRotation().getDegrees();
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

    LimelightHelpers.Results results = 
        LimelightHelpers.getLatestResults(VisionConstants.tagLimelight).targetingResults;

    if(LimelightHelpers.getTV(VisionConstants.tagLimelight)){
      Pose2d camPose = LimelightHelpers.toPose2D(results.botpose_wpiblue);
      m_drive.addVisionMeasurement(camPose, 
      Timer.getFPGATimestamp() - (results.latency_capture / 1000.0)
       - (results.latency_pipeline / 1000.0));
      m_field.getObject("Cam est Pose").setPose(camPose);
    } else {
      m_field.getObject("Cam est Pose").setPose(m_drive.getState().Pose);
    }

    m_field.setRobotPose(m_drive.getState().Pose);
  }

  public void setDynamicVisionStdDevs(){
    int numDetectedTargets = getNumofDetectedTargets();
    double stdsDevX = 0.0;
    double stdsDevY = 0.0;
    
    double stdsDevDeg = 0.0;

    if(DriverStation.isAutonomous()){
      if(allowedToFilterAuto()){
       stdsDevX = 0.1;
       stdsDevY = 0.1;
       stdsDevDeg = 0.1;
      } else {
        stdsDevX = 100.0;
        stdsDevY = 100.0;
       stdsDevDeg = 100.0;
      }
    } else {
      if(numDetectedTargets <= 2){
      stdsDevX = 0.1;
      stdsDevY = 0.1;
      stdsDevDeg = 0.1;
     } else {
      stdsDevX = Math.abs(m_drive.getState().speeds.vxMetersPerSecond) * 20;
      stdsDevY = Math.abs(m_drive.getState().speeds.vyMetersPerSecond) * 20;
      stdsDevDeg = Math.abs(m_drive.getState().speeds.omegaRadiansPerSecond) * 20;
     }
    }

    Matrix<N3, N1> visionMat = MatBuilder.fill(Nat.N3(), Nat.N1(), stdsDevX, stdsDevY, stdsDevDeg);

    m_drive.setVisionMeasurementStdDevs(visionMat);
  }

  public double getDistanceToTarget(){
    double distance = 0.0;
    
    distance = LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.tagLimelight).getZ();

    return distance;
  }

  public void setCameraPipeline(int pipeline){
    LimelightHelpers.setPipelineIndex(VisionConstants.tagLimelight, pipeline);
  }
  
  public int getNumofDetectedTargets(){
    return LimelightHelpers
    .getLatestResults(VisionConstants.tagLimelight).targetingResults.targets_Fiducials.length;
  }

  public boolean allowedToFilterAuto(){
    if(m_drive.getCurrentRobotChassisSpeeds().vxMetersPerSecond < 0.1 &&
       getNumofDetectedTargets() >= 2){
      return true;
    } else {
      return false;
    }
  }
}