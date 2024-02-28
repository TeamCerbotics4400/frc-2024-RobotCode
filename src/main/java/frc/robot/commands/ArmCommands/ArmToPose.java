// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.POVSelector;

public class ArmToPose extends Command {
  /** Creates a new ArmToPose. */
  ArmSubsystem m_arm;
  POVSelector m_selector;
  double angle = 0.0;

  public ArmToPose(ArmSubsystem m_arm, POVSelector m_selector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_selector = m_selector;

    addRequirements(m_arm);
  }

  public ArmToPose(ArmSubsystem m_arm2, Object object, POVSelector m_selector2) {
    //TODO Auto-generated constructor stub
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_arm.isEnabled()){m_arm.enable();}
   m_arm.getController().reset(m_arm.getMeasurement());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(LimelightHelpers.getTV(VisionConstants.tagLimelightName) == true || LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.tagLimelightName).getZ() <= 4.35){
         angle = m_arm.getAngleForDistance(LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.tagLimelightName).getZ());
        }
       else {
           angle = 160.0;
        } 
             m_arm.updateArmSetpoint(angle);

      }  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.updateArmSetpoint(160.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

