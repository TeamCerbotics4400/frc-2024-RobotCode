// Code stolen by me

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.limelightOffsets;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveTrain;

public class AutoAlignTest extends Command {

  private final DriveTrain drivetrain; 
    private final LimelightHelpers limelight;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final PIDController drivePID, strafePID, rotationPID;
    private final boolean alingToAprilTag;
    private final limelightOffsets offsets;
    private final double driveOffset, strafeOffset, rotationOffset;

  /** Creates a new AutoAlignTest. */
  public AutoAlignTest(DriveTrain driveTrain, LimelightHelpers limelight, boolean alingToAprilTag) {
    this.drivetrain = driveTrain;
    this.limelight = limelight;


    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    this.drivePID = new PIDController(
      Constants.kPdrive, 
      Constants.kIdrive, 
      Constants.kDdrive);

  this.strafePID = new PIDController(
      Constants.kPstrafe, 
      Constants.kIstrafe, 
      Constants.kDstrafe);

  this.rotationPID = new PIDController(
      Constants.kProtation, 
      Constants.kIrotation, 
      Constants.kDrotation);

      this.alingToAprilTag = alingToAprilTag;

      this.offsets = limelight.getOffsets(alingToAprilTag);  

      this.driveOffset = offsets.driveOffset;
      this.strafeOffset = offsets.strafeOffset;
      this.rotationOffset = offsets.rotationOffset;
      
      addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
