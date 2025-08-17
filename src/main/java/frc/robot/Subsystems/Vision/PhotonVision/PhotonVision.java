package frc.robot.Subsystems.Vision.PhotonVision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class PhotonVision extends SubsystemBase {

  List<Pair<PhotonVisionIO, PhotonVisionIOInputsAutoLogged>> photonVisionInputAndOutput =
      new ArrayList<>();

  public PhotonVision(PhotonVisionIO... ios) {
    for (PhotonVisionIO io : ios)
      photonVisionInputAndOutput.add(Pair.of(io, new PhotonVisionIOInputsAutoLogged()));
  }

  public PhotonVisionIOInputsAutoLogged getInputsFromPhotonVisionName(String photonVisionName) {
    return photonVisionInputAndOutput.stream()
        .filter(
            cameraPair ->
                cameraPair.getFirst().getPhotonVisionCameraName().equals(photonVisionName))
        .findFirst()
        .get()
        .getSecond();
  }

  public PhotonVisionIO getPhotonVisionIOFromPhotonVisionName(String photonVisionName) {
    return photonVisionInputAndOutput.stream()
        .filter(
            cameraPair ->
                cameraPair.getFirst().getPhotonVisionCameraName().equals(photonVisionName))
        .findFirst()
        .get()
        .getFirst();
  }

  public List<ObjectRectangle> getListOfObjectRectangles(String photonVisionName) {
    List<ObjectRectangle> objectRectangles = new ArrayList<>();
    var cornersArray =
        this.getInputsFromPhotonVisionName(photonVisionName).minAreaRectCornersOfTargets;
    // 4 corners of the rectangle
    int amountOfTargets = cornersArray.length;
    for (int i = 0; i < amountOfTargets; i++) {
      objectRectangles.add(new ObjectRectangle(cornersArray[i]));
    }

    return objectRectangles;
  }

  public List<Double> getDistancesToTargetsUsingHeight(String cameraName) {
    List<Double> distances = new ArrayList<>();
    var targets = this.getListOfObjectRectangles(cameraName);

    for (ObjectRectangle obj : targets) {
      double distance =
          (PhotonVisionConstants.PhysicalConstants.CAMERA_FOCAL_Y
                  * PhotonVisionConstants.PhysicalConstants.CORAL_HEIGHT)
              / obj.getObjectPixelHeight();
      distances.add(distance);
    }

    return distances;
  }

  public List<Double> getDistancesToTargetsUsingWidth(String cameraName) {
    List<Double> distances = new ArrayList<>();
    var targets = this.getListOfObjectRectangles(cameraName);

    for (ObjectRectangle obj : targets) {
      double distance =
          (PhotonVisionConstants.PhysicalConstants.CAMERA_FOCAL_X
                  * PhotonVisionConstants.PhysicalConstants.CORAL_WIDTH)
              / obj.getObjectPixelWidth();
      distances.add(distance);
    }

    return distances;
  }

  public List<Double> getAvgDistanceToTargets(String cameraName) {
    List<Double> avgDistances = new ArrayList<>();
    var heightBasedDistances = this.getDistancesToTargetsUsingHeight(cameraName);
    var widthBasedDistances = this.getDistancesToTargetsUsingWidth(cameraName);
    for (int i = 0; i < heightBasedDistances.size(); i++) {
      avgDistances.add((heightBasedDistances.get(i) + widthBasedDistances.get(i)) / 2);
    }
    return avgDistances;
  }

  public List<Double> getPerpendicularDistanceToTargets(String cameraName) {
    List<Double> perpDistances = new ArrayList<>();

    var distances = this.getAvgDistanceToTargets(cameraName);
    var yawes = this.getInputsFromPhotonVisionName(cameraName).yawOfTargets;

    for (int i = 0; i < distances.size(); i++) {
      perpDistances.add(distances.get(i) * Math.sin(Units.degreesToRadians(yawes[i])));
    }

    return perpDistances;
  }

  public List<Double> getParallelDistanceToTargets(String cameraName) {
    List<Double> parallelDistances = new ArrayList<>();

    var distances = this.getAvgDistanceToTargets(cameraName);
    var pitches = this.getInputsFromPhotonVisionName(cameraName).picthOfTargets;

    for (int i = 0; i < distances.size(); i++) {
      parallelDistances.add(
          distances.get(i)
              * Math.cos(
                  Units.degreesToRadians(pitches[i])
                      + PhotonVisionConstants.PositionalConstants.mountingAngle));
    }

    return parallelDistances;
  }

  public List<Pose2d> getListOfTargetPoses(String cameraName) {
    List<Pose2d> targetPoses = new ArrayList<>();
    for (int i = 0; i < this.getInputsFromPhotonVisionName(cameraName).amountOfTargets; i++) {
      Pose2d currentPose = RobotContainer.m_Drivetrain.getPose2d();

      List<Double> paraDistancesToTargets = this.getParallelDistanceToTargets(cameraName);
      List<Double> anglesToTargets =
          Arrays.stream(this.getInputsFromPhotonVisionName(cameraName).yawOfTargets)
              .boxed()
              .collect(Collectors.toList());

      double xComp = currentPose.getX();
      double yComp = currentPose.getY();

      double botHeading = currentPose.getRotation().getDegrees();

      targetPoses.add(
          new Pose2d(
              new Translation2d(
                  xComp
                      + paraDistancesToTargets.get(i)
                          * Math.cos(Units.degreesToRadians(botHeading + anglesToTargets.get(i))),
                  yComp
                      + paraDistancesToTargets.get(i)
                          * Math.sin(Units.degreesToRadians(botHeading + anglesToTargets.get(i)))),
              new Rotation2d()));
    }
    return targetPoses;
  }

  public Pose2d[] getArrayOfTargetPoses(String cameraName) {
    var poseList = this.getListOfTargetPoses(cameraName);
    Pose2d[] poseArray = new Pose2d[poseList.size()];
    for (int i = 0; i < poseArray.length; i++) {
      poseArray[i] = poseList.get(i);
    }
    return poseArray;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < photonVisionInputAndOutput.size(); i++) {

      var curPair = photonVisionInputAndOutput.get(i);

      var io = curPair.getFirst();
      var input = curPair.getSecond();
      input.posesOfTargets = this.getArrayOfTargetPoses(io.getPhotonVisionCameraName());

      io.updateInputs(input);
      Logger.processInputs(io.getPhotonVisionCameraName(), input);
    }
  }
}
