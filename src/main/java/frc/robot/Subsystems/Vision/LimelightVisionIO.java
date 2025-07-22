package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface LimelightVisionIO {
    
    default void updateInputs(VisionIOInputs inputs) {}

    default void setLeds(boolean on) {}

    default void setLimelightLocation(Pose3d limelightPose) {}

    default Pose3d getLimelightLocation() {
        return new Pose3d();
    }

    default String getLimelightName() {
        return "";
    }

    default void setThrottleValue(int throttleValue) {}

    @AutoLog
    class VisionIOInputs {
        public boolean hasTargets = false;
        public int tagId = -1;
        public boolean connected;
        public double distanceToTagMeters;
        public double yaw;
        public double pitch;
        public double area;
        public Pose2d visionPoseEstimate;
    }
}
