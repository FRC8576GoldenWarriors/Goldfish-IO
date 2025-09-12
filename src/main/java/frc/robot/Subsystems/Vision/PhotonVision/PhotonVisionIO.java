package frc.robot.Subsystems.Vision.PhotonVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionIO implements PhotonVisionLoggedIO {

  private PhotonCamera camera;
  private PhotonPoseEstimator pvEstimator;

  public PhotonVisionIO(String cameraName, Pose3d cameraPose) {
    camera = new PhotonCamera(cameraName);
    pvEstimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraPose.minus(new Pose3d(0, 0, 0, new Rotation3d())));
  }

  @Override
  public String getPhotonVisionCameraName() {
    return camera.getName();
  }

  public Matrix<N3, N3> getCameraMatrix() {
    return camera.getCameraMatrix().get();
  }

  @Override
  public synchronized void updateInputs(PhotonVisionIOInputs inputs) {
    inputs.isConnected = camera.isConnected();
    pvEstimator.addHeadingData(
        Timer.getFPGATimestamp(),
        new Rotation2d(RobotContainer.m_Drivetrain.getBlueAbsoluteHeading()));
    if (inputs.isConnected) {
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();
      inputs.hasResults = results.size() > 0;
      if (inputs.hasResults) {
        PhotonPipelineResult result = results.get(results.size() - 1);

        pvEstimator
            .update(result)
            .ifPresent(
                (pose) ->
                    RobotContainer.m_Drivetrain.addVisionMeasurement(
                        pose.estimatedPose.toPose2d(), pose.timestampSeconds));

        inputs.hasTargets = result.hasTargets();
        if (inputs.hasTargets) {
          List<PhotonTrackedTarget> targets = result.getTargets();

          inputs.objectIDOfTargets = new int[targets.size()];
          inputs.yawOfTargets = new double[targets.size()];
          inputs.picthOfTargets = new double[targets.size()];
          inputs.skewOfTargets = new double[targets.size()];
          inputs.areaOfTargets = new double[targets.size()];
          inputs.ambiguityOfTargets = new double[targets.size()];
          inputs.minAreaRectCornersOfTargets = new double[targets.size()][8];

          inputs.amountOfTargets = targets.size();

          for (int i = 0; i < targets.size(); i++) {
            inputs.objectIDOfTargets[i] = targets.get(i).getDetectedObjectClassID();
            inputs.yawOfTargets[i] = targets.get(i).getYaw();
            inputs.picthOfTargets[i] = targets.get(i).getPitch();
            inputs.skewOfTargets[i] = targets.get(i).getSkew();
            inputs.areaOfTargets[i] = targets.get(i).getArea();
            inputs.ambiguityOfTargets[i] = targets.get(i).getDetectedObjectConfidence();

            for (int j = 0; j < 8; j += 2) {
              inputs.minAreaRectCornersOfTargets[i][j] =
                  targets.get(i).getMinAreaRectCorners().get(j / 2).x;
              inputs.minAreaRectCornersOfTargets[i][j + 1] =
                  targets.get(i).getMinAreaRectCorners().get(j / 2).y;
            }
          }
        }
      }
    }
  }
}
