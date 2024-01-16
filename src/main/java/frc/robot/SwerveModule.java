// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import team4400.Util.Swerve.CANModuleOptimizer;
import team4400.Util.Swerve.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {

    public final int moduleNumber;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;

    private final AnalogEncoder absoluteEncoder;

    private final SimpleMotorFeedforward feedForward;

    private final SparkPIDController driveController;
    private final PIDController turnController;
 
    private final double absoluteEncoderOffset;

    private Rotation2d lastAngle;
    
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){

        this.moduleNumber = moduleNumber;

        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(moduleConstants.turnMotorID, MotorType.kBrushless);



        absoluteEncoderOffset = moduleConstants.angleOffset;
        absoluteEncoder = new AnalogEncoder(moduleConstants.absoluteEncoderID);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setInverted(moduleConstants.driveReversed);
        turnMotor.setInverted(moduleConstants.turnReversed);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kCoast);

        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        feedForward = new 
            SimpleMotorFeedforward(ModuleConstants.kS, ModuleConstants.kV, ModuleConstants.kA);

        driveController = driveMotor.getPIDController();

        driveController.setP(ModuleConstants.kP);
        driveController.setI(ModuleConstants.kI);
        driveController.setD(ModuleConstants.kD);
        driveController.setFF(ModuleConstants.kFF);

        turnController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        Timer.delay(1.0);
        resetEncoders();

        lastAngle = getState().angle;
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getRawAbsoluteVolts(){
        return absoluteEncoder.getAbsolutePosition();
    }

    public double getAngleDeegrees(){
        double rawAngle = 
                (absoluteEncoder.getAbsolutePosition() * 360 - absoluteEncoderOffset) % 360;
        double angle;
        if(rawAngle > 180.0 && rawAngle < 360.0){
            angle = -180 + rawAngle % 180.0;
        } else {
            angle = rawAngle;
        }

        return angle;
    }

    public double turningDeegreesToRadians(){
        return Units.degreesToRadians(getAngleDeegrees());
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        absoluteEncoder.reset();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), 
        Rotation2d.fromDegrees(getAngleDeegrees()));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){

        desiredState = 
            CANModuleOptimizer.optimize(desiredState, getState().angle);


        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);  

        SmartDashboard.putString("Swerve [" + moduleNumber + "] state", desiredState.toString());
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = 
                desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 
            0, feedForward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= 
        (DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.01)) ? lastAngle : desiredState.angle;

        turnMotor
        .set(turnController.calculate(turningDeegreesToRadians(), desiredState.angle.getRadians()));
        lastAngle = angle;
    }

    public void lockModule(){
        double targetAngle = -45;
        turnMotor.set(turnController.calculate(targetAngle));
    }

    //Invert if needed for odometry correction
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(), 
            Rotation2d.fromDegrees(getAngleDeegrees()));
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    //Debug
    public void tuneModulePID(double speedMtsPerSec){
        driveController.setReference(speedMtsPerSec, ControlType.kVelocity, 
            0, feedForward.calculate(speedMtsPerSec));
    }
}