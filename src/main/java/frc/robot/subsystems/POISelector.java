/* Wont use it, keeping it in case

/ Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class POISelector extends SubsystemBase {

  private ArrayList<String> shootingLevels = new ArrayList<String>();
  private int currentShooterSelection;
  Joystick joy;

  public POISelector(Joystick joy) {
    this.joy = joy;
    this.currentShooterSelection = 1;

    shootingLevels.add("AMP");
    shootingLevels.add("Speaker");
  }

  @Override
  public void periodic() {
      displaySelection();
  }

    public void displaySelection(){
    
    String currentKeyLevels = shootingLevels.get(currentShooterSelection);

    if (currentKeyLevels != null) {
      // Get the string representation of the selected entry
  
      // Display the selected entry on the SmartDashboard
      SmartDashboard.putString("Selected Shooter", currentKeyLevels);

    } else {
      // Display a message indicating that the selected entry is null
      SmartDashboard.putString("Selected Entry", "No node selected");
    }
  }

  public void updateSelectionUp(){
    int pov = joy.getPOV();

    if(pov == 0){
      currentShooterSelection++;
      if(currentShooterSelection >= shootingLevels.size()){
        currentShooterSelection = 0;
      }
    }  
  }

  public void updateSelectionDown(){
    int pov = joy.getPOV();

    if(pov == 180){
      currentShooterSelection--;
      if(currentShooterSelection < 0){
        currentShooterSelection = shootingLevels.size() - 1;
      }
    }  
  }
  
  public String getShooterName(){
    String currentKey = shootingLevels.get(currentShooterSelection);
    return currentKey;
  }
}
*/