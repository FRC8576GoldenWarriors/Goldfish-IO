package frc.robot.Subsystems.Vision.PhotonVision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

interface PhotonVisionLoggedIO {

  default String getPhotonVisionCameraName() {
    return "";
  }

  default void updateInputs(PhotonVisionIOInputs inputs) {}

  @AutoLog
  class PhotonVisionIOInputs {
    boolean isConnected = false;
    boolean hasResults = false;
    boolean hasTargets = false;

    int amountOfTargets = 0;

    int[] objectIDOfTargets = new int[0];

    double[] yawOfTargets = new double[0];
    double[] picthOfTargets = new double[0];
    double[] skewOfTargets = new double[0];
    double[] areaOfTargets = new double[0];
    double[] ambiguityOfTargets = new double[0];
    Pose2d[] posesOfTargets = new Pose2d[0];

    double[][] minAreaRectCornersOfTargets = new double[0][0];
  }
}
