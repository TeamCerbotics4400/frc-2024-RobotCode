// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AligningCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import team4400.Util.Swerve.FieldCentricAiming;

public class VelocityOffset extends Command {

    CommandSwerveDrivetrain m_drivetrain;
    FieldCentricAiming m_FieldCentricAiming;

    boolean m_isDone;

    Timer shotTimer;
    Boolean ranOnce;

    Pose2d currentRobotPose;
    Translation2d currentRobotTranslation;
    Rotation2d currentAngleToSpeaker;

    Pose2d futureRobotPose2d;
    Translation2d futureRobotTranslation;
    Rotation2d futureAngleToSpeaker;

    ChassisSpeeds speeds;
    Translation2d moveDelta;

    /**The calculated the time until the note leaves based on the constant and time since button press */
    Double timeUntilShot; 
    DoubleSupplier m_trigger; 

    Double correctedDistance;
    Rotation2d correctedRotation;

    /** Calculates the velocity compensated target to shoot at 
     * @param drivetrain CommandSwerveDrivetrain instance
     * @param triggerAxis The trigger axis on which shoot is bound
     * @see FieldCentricAiming
    */
    public VelocityOffset(CommandSwerveDrivetrain drivetrain, DoubleSupplier triggerAxis) {
        m_drivetrain = drivetrain;
        m_trigger = triggerAxis;
        shotTimer = new Timer();
        ranOnce = false;
        m_FieldCentricAiming = new FieldCentricAiming();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //Starts shot timer after trigger press
        //Get current translation of the drivetrain
        currentRobotTranslation = m_drivetrain.getState().Pose.getTranslation(); 
        //Calculate angle relative to the speaker from current pose
        currentAngleToSpeaker = m_FieldCentricAiming.getAngleToSpeaker(currentRobotTranslation); 
        //Get current drivetrain velocities in field relative terms
        speeds = m_drivetrain.getFieldRelativeChassisSpeeds(); 
        
        timeUntilShot = Constants.ShooterConstants.kTimeToShoot - shotTimer.get();
        if (timeUntilShot < 0) {
            timeUntilShot = 0.00;
        }

        //Calculate change in x/y distance due to time and velocity
        moveDelta = new Translation2d(Constants.ShooterConstants.kTimeToShoot*(speeds.vxMetersPerSecond),Constants.ShooterConstants.kTimeToShoot*(speeds.vyMetersPerSecond));

        //futureRobotPose is the position the robot will be at timeUntilShot in the future
        futureRobotTranslation = currentRobotTranslation.plus(moveDelta);
        //Angle to the speaker at future position
        futureAngleToSpeaker = m_FieldCentricAiming.getAngleToSpeaker(futureRobotTranslation);

        //The amount to add to the current angle to speaker to aim for the future
        correctedRotation = futureAngleToSpeaker;
        //correctedRotation = currentAngleToSpeaker; //Uncomment to disable future pose aiming
        // Get the future distance to speaker
        correctedDistance = m_FieldCentricAiming.getDistToSpeaker(futureRobotTranslation);
        m_drivetrain.setVelocityOffset(correctedRotation); //Pass the offsets to the drivetrain

        //SmartDashboard.putString("SI LLEGO ACÁ", "si llego we");  
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shotTimer.stop();
        shotTimer.reset();
        //m_isDone = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }

}