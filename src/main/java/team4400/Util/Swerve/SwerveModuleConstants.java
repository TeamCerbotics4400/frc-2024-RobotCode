// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team4400.Util.Swerve;

import com.ctre.phoenix6.signals.InvertedValue;

/** Add your docs here. */
/* Special class for helping us organize Swerve Module Constants */
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int turnMotorID;
    public final InvertedValue driveReversed;
    public final InvertedValue turnReversed;
    public final int absoluteEncoderID;
    public final double angleOffset;

    public SwerveModuleConstants(int driveMotorID, int turnMotorID,
    InvertedValue driveReversed, InvertedValue turnReversed, 
    int absoluteEncoderID, double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.driveReversed = driveReversed;
        this.turnReversed = turnReversed;
        this.absoluteEncoderID = absoluteEncoderID;
        this.angleOffset = angleOffset;
    }
}
