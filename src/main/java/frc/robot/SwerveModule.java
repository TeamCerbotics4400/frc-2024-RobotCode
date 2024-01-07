// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import team4400.Util.Conversions;
import team4400.Util.Swerve.CANModuleOptimizer;
import team4400.Util.Swerve.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {

    public final int moduleNumber;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final AnalogEncoder absoluteEncoder;
 
    private final double absoluteEncoderOffset;

    private final SimpleMotorFeedforward feedForward;

    private Rotation2d lastAngle;

    public TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    public TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final PositionVoltage anglePosition = new PositionVoltage(0);
    
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){

        this.moduleNumber = moduleNumber;

        driveMotor = new TalonFX(moduleConstants.driveMotorID, "rio");
        turnMotor = new TalonFX(moduleConstants.turnMotorID, "rio");

        absoluteEncoderOffset = moduleConstants.angleOffset;
        absoluteEncoder = new AnalogEncoder(moduleConstants.absoluteEncoderID);

        feedForward = new 
            SimpleMotorFeedforward(ModuleConstants.kS, ModuleConstants.kV, ModuleConstants.kA);
        
        /* Drive Motor */
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.MotorOutput.Inverted = moduleConstants.driveReversed;

        driveMotorConfig.Slot0.kP = ModuleConstants.kP;
        driveMotorConfig.Slot0.kI = ModuleConstants.kI;
        driveMotorConfig.Slot0.kD = ModuleConstants.kD;

        /* Turn Motor */
        turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnMotorConfig.MotorOutput.Inverted = moduleConstants.turnReversed;

        turnMotorConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kTurningMotorGearRatio;
        turnMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

        turnMotorConfig.Slot0.kP = ModuleConstants.kPTurning;
        turnMotorConfig.Slot0.kI = 0;
        turnMotorConfig.Slot0.kD = 0;

        driveMotor.getConfigurator().apply(driveMotorConfig);
        turnMotor.getConfigurator().apply(turnMotorConfig);

        Timer.delay(1.0);
        resetEncoders();

        lastAngle = getState().angle;
    }

    public double getDrivePosition(){
        return Conversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(), 
                                                            ModuleConstants.kWheelCircumerence);
    }

    public double getDriveVelocity(){
        return Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), 
                                                            ModuleConstants.kWheelCircumerence);
    }

    public double getTurnRotation(){
        return turnMotor.getPosition().getValueAsDouble();
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
        driveMotor.setPosition(0);
        turnMotor.setPosition(Units.degreesToRotations(getAngleDeegrees()));//absoluteEncoder.reset();
    }   

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), 
        Rotation2d.fromRotations(getTurnRotation()));
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
            driveDutyCycle.Output = 
                desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            driveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = 
                            Conversions.MPSToRPS(desiredState.speedMetersPerSecond, 
                                                ModuleConstants.kWheelCircumerence);
            driveVelocity.FeedForward = feedForward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= 
        (DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.01)) ? lastAngle : desiredState.angle;

        
        turnMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        lastAngle = angle;
    }

    public void lockModule(){
        double targetAngle = -45;
        //turnMotor.set(turnController.calculate(targetAngle));
    }

    //Invert if needed for odometry correction
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(), 
            Rotation2d.fromDegrees(getAngleDeegrees()));
    }

    public void stop(){
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    //Debug
    public void tuneModulePID(double speedMtsPerSec){
        var request = new VelocityVoltage(0).withSlot(0);
        driveMotor.setControl(request.withVelocity(speedMtsPerSec));
    }
}
