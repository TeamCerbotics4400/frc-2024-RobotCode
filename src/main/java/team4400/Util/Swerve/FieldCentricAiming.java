package team4400.Util.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class FieldCentricAiming {
    private Pose2d defaultPosition = new Pose2d(new Translation2d(0,0), new Rotation2d(0));
    private Pose2d speakerPosition = defaultPosition;
    
    public FieldCentricAiming() {
        try {
            getSpeakerPos();
        } catch (Exception e) {
            // TODO: handle exception
        }
        
    }

    public Pose2d getSpeakerPos() {
        if (speakerPosition == defaultPosition) {
            if (DriverStation.getAlliance() != null && !DriverStation.getAlliance().isEmpty()) {
                speakerPosition = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Constants.FieldConstants.BLUE_SPEAKER : Constants.FieldConstants.RED_SPEAKER;
            }
        }
        return speakerPosition;
    }

    public double getDistToSpeaker(Translation2d robotPose) {
        return getSpeakerPos().getTranslation().getDistance(robotPose);
    }

    public Rotation2d getAngleToSpeaker(Translation2d pose) {
        return getSpeakerPos().getTranslation().minus(pose).getAngle();
    }

}