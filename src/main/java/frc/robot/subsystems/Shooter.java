// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  //upper
  CANSparkMax leftFlyWheel = new CANSparkMax (6,MotorType.kBrushless);
  CANSparkMax rightFlyWheel = new CANSparkMax (1,MotorType.kBrushless);
//lower
  RelativeEncoder rightEncoder;
  RelativeEncoder leftEncoder;

  SparkPIDController leftController; 
  SparkPIDController rightController; 

  private double leftVelocity = 0;
  private double rightVelocity = 0;

  double leftkP = 0.000121, leftkI = 0, leftkD = 0, leftkFF = 0.0002,
         rightkP = 0.000121, rightkI = 0, rightkD = 0, rightkFF = 0.0002;
  double leftSetPoint = 0, rightSetPoint = 0;

  public Shooter() {
    leftFlyWheel.restoreFactoryDefaults();
    rightFlyWheel.restoreFactoryDefaults();

    rightFlyWheel.setInverted(true);
    leftFlyWheel.setInverted(true);

    rightFlyWheel.setIdleMode(IdleMode.kBrake);
    leftFlyWheel.setIdleMode(IdleMode.kBrake);

    rightFlyWheel.clearFaults();
    leftFlyWheel.clearFaults();

    leftController = leftFlyWheel.getPIDController();
    rightController = rightFlyWheel.getPIDController();

    leftEncoder = leftFlyWheel.getEncoder();
    rightEncoder = rightFlyWheel.getEncoder();

    SmartDashboard.putNumber("left kP", leftkP);   SmartDashboard.putNumber("right kP", rightkP);
    SmartDashboard.putNumber("left kI", leftkI);   SmartDashboard.putNumber("right kI", rightkI);
    SmartDashboard.putNumber("left kD", leftkD);   SmartDashboard.putNumber("right kD", rightkD);
    SmartDashboard.putNumber("left kFF", leftkFF);   SmartDashboard.putNumber("right kFF", rightkFF);

    SmartDashboard.putNumber("left RPM", leftSetPoint);   SmartDashboard.putNumber("right RPM", rightSetPoint);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     double ukP = SmartDashboard.getNumber("left kP", 0.02),
            ukI = SmartDashboard.getNumber("left kI", 0),
            ukD = SmartDashboard.getNumber("left kD", 0.000190), 
            lkP = SmartDashboard.getNumber("right kP", 0.02),
            lkI = SmartDashboard.getNumber("right kI", 0),
            lkD = SmartDashboard.getNumber("right kD", 0.000190),
            ukFF = SmartDashboard.getNumber("left kFF",0.114),
            lkFF = SmartDashboard.getNumber("right kFF",0.114); 

    if (leftkP != ukP){ leftkP = ukP; leftController.setP(leftkP); }
    if (leftkI != ukI){ leftkI = ukI; leftController.setI(leftkI); }
    if (leftkD != ukD){ leftkD = ukD; leftController.setD(leftkD); }
    if (leftkFF != ukFF){ leftkFF = ukFF; leftController.setFF(leftkFF);}
    if (rightkP != lkP){ rightkP = lkP; rightController.setP(rightkP); }
    if (rightkI != lkI){ rightkI = lkI; rightController.setI(rightkI); }
    if (rightkD != lkD){ rightkD = lkD; rightController.setD(rightkD);}
    if (rightkFF != lkFF){ rightkFF = lkFF; rightController.setFF(rightkFF);}

      double uRPM = SmartDashboard.getNumber("left RPM", 0),
             lRPM = SmartDashboard.getNumber("right RPM", 0);

    if (leftSetPoint != uRPM){leftSetPoint = uRPM;}
    if (rightSetPoint != lRPM){rightSetPoint = lRPM;}

    SmartDashboard.putNumber("Current left RPM", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Current right RPM", rightEncoder.getVelocity());

  }


  public void setleftSpeed(){
    leftVelocity = leftSetPoint;
    leftController.setReference(leftVelocity,ControlType.kVelocity);
  }
  public void setrightSpeed(){
    rightVelocity = rightSetPoint;
    rightController.setReference(rightVelocity,ControlType.kVelocity);
  }
    public void stopleft(){
    leftFlyWheel.set(0);
  }

  public void stopright(){
    rightFlyWheel.set(0);
  }
}
