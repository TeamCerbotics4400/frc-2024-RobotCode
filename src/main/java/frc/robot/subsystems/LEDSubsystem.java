// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private Spark ledController = new Spark(9);
  private Timer animationTimer = new Timer();
  private boolean blueActive = false;
  public LEDSubsystem() {}

  @Override
  public void periodic() {
  }

  public void setBlue(){
    ledController.set(0.87);
  }

  public void setGreen(){
    ledController.set(0.77);
  }

  public void strobeColor1(){
    ledController.set(0.15);
  }

  public void breathColor1(){
    ledController.set(0.11);
  }

  public void strobeColor2(){
    ledController.set(0.35);
  }

  public void breathColor2(){
    ledController.set(0.31);
  }

  public void strobeBlue(){
    ledController.set(-0.09);
  }

  public void noteIntaked(){
    
    animationTimer.start();

    if(!animationTimer.hasElapsed(2)){
      breathColor1();
    } else {
      animationTimer.stop();
      animationTimer.reset();
     
    }
  }

  public void setBlueFalse(){
    blueActive = false;
  }

  public void setBlueTrue(){
    blueActive = true;
  }

  public boolean getBlueState(){
    return blueActive;
  }

}
