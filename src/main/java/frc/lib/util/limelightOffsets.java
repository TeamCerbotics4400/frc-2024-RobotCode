/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

 package frc.lib.util;

 public class limelightOffsets {
     
     public double driveOffset;
     public double strafeOffset;
     public double rotationOffset;
 
     public limelightOffsets(
        double driveValue,
        double strafeValue,
        double rotationOffset
         
     ){
         this.driveOffset = driveValue;
         this.strafeOffset = strafeValue;
         this.rotationOffset = rotationOffset;
         
 
     }
 }
 