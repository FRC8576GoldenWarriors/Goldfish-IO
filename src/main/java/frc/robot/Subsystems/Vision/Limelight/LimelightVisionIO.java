package frc.robot.Subsystems.Vision.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

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
    public boolean connected = false;
    public double yaw = 0;
    public double pitch = 0;
    public double area = 0;

    public boolean isAligned = false;
    public double[] generatedStddevs = null;
    public double timeInSeconds = 0.0;
    public double velocity = 0.0;
  }
}
