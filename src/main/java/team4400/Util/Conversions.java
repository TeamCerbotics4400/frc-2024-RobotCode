// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team4400.Util;

/** Add your docs here. */
public class Conversions {
  public static double MPSToRPS(double wheelMPS, double circumference){
    double wheelRPS = wheelMPS / circumference;
    return wheelRPS;
  }

  public static double RPSToMPS(double wheelRPS, double circumference){
    double wheelMPS = wheelRPS * circumference;
    return wheelMPS;
  }

  public static double rotationsToMeters(double wheelRotations, double circumference){
    double wheelMeters = wheelRotations * circumference;
    return wheelMeters;
  }

  public static double metersToRotations(double wheelMeters, double circumference){
    double wheelRotations = wheelMeters / circumference;
    return wheelRotations;
  }

  public static double rotationsToDegrees(double motorRotation){
    double degrees = motorRotation * 360;
    return degrees;
  }

  public static double degreesToRotations(double motorRotation){
    double rotations = motorRotation / 360;
    return rotations;
  }
}

