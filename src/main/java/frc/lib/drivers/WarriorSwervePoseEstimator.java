package frc.lib.drivers;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystems.Vision.Limelight.LimelightConstants;
import frc.robot.Subsystems.Vision.Limelight.LimelightConstants.NameConstants;
import frc.robot.Subsystems.Vision.Limelight.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.Vision.Limelight.LimelightHelpers.RawFiducial;
import frc.robot.Subsystems.Vision.Limelight.LimelightIO;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class WarriorSwervePoseEstimator extends SwerveDrivePoseEstimator implements Periodical {

  List<Supplier<Pair<String, PoseEstimate>>> poseEstimateSuppliers = new ArrayList<>();
  Supplier<Double> currentTimeSeconds;
  Supplier<Rotation2d> gyroAngle;
  Supplier<SwerveModulePosition[]> wheelPositions;
  Supplier<ChassisSpeeds> robotSpeeds;
  List<RawFiducial> currentVisibleTags;

  Queue<Pair<String, PoseEstimate>> poseEstimateQueue = new LinkedList<>();

  boolean useDynamicVisionDeviations;
  AprilTagFieldLayout fieldMap;

  public WarriorSwervePoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters,
      AprilTagFields aprilTagField,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs,
      boolean useDynamicVisionDeviations) {
    super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    this.fieldMap = AprilTagFieldLayout.loadField(aprilTagField);
    this.useDynamicVisionDeviations = useDynamicVisionDeviations;
    PeriodicalUtil.registerPeriodic(this);
  }

  public WarriorSwervePoseEstimator withDriveUpdates(
      Supplier<Double> currentTimeSeconds,
      Supplier<Rotation2d> gyroAngle,
      Supplier<SwerveModulePosition[]> wheelPositions) {
    this.currentTimeSeconds = currentTimeSeconds;
    this.gyroAngle = gyroAngle;
    this.wheelPositions = wheelPositions;
    return this;
  }

  public WarriorSwervePoseEstimator withRobotSpeeds(Supplier<ChassisSpeeds> robotSpeeds) {
    this.robotSpeeds = robotSpeeds;
    return this;
  }

  public void addPoseEstimateUpdaters(Supplier<Pair<String, PoseEstimate>> poseEstimateSupplier) {
    poseEstimateSuppliers.add(poseEstimateSupplier);
  }

  public boolean isValidPoseEstimate(PoseEstimate poseEstimate) {

    double maximumTagAmbiguity = 0.4;

    if (poseEstimate == null) return false;

    if (poseEstimate.pose == null) return false;

    if (poseEstimate.tagCount < 1) return false;

    if (poseEstimate.avgTagDist < 0.1 || poseEstimate.avgTagDist >= 5) return false;

    if (poseEstimate.pose.getX() < 0 || poseEstimate.pose.getX() > fieldMap.getFieldLength())
      return false;

    if (poseEstimate.pose.getY() < 0 || poseEstimate.pose.getY() > fieldMap.getFieldWidth())
      return false;

    if (poseEstimate.rawFiducials.length < 1) return false;

    double averageTagAmbiguity =
        Arrays.stream(poseEstimate.rawFiducials)
                .mapToDouble((tag) -> tag.ambiguity)
                .sum()
            / poseEstimate.rawFiducials.length;

    if (averageTagAmbiguity > maximumTagAmbiguity) return false;

    ChassisSpeeds currentRobotSpeeds = robotSpeeds.get();

    if (currentRobotSpeeds.omegaRadiansPerSecond > 4 * Math.PI) return false;

    if (Math.sqrt(
            Math.pow(currentRobotSpeeds.vxMetersPerSecond, 2)
                + Math.pow(currentRobotSpeeds.vyMetersPerSecond, 2))
        > 6) return false;

    return true;
  }

  public boolean isValidPoseEstimate(Pair<String, PoseEstimate> poseEstimatePair) {
    return this.isValidPoseEstimate(poseEstimatePair.getSecond());
  }

  private Matrix<N3, N1> generateDynamicVisionDeviations(
      PoseEstimate poseEstimate, ChassisSpeeds drivetrainSpeeds) {

    double generatedXDev =
        (poseEstimate.avgTagDist + Math.abs(drivetrainSpeeds.vxMetersPerSecond))
            / Math.pow(poseEstimate.tagCount, 2);
    double generatedYDev =
        (poseEstimate.avgTagDist + Math.abs(drivetrainSpeeds.vyMetersPerSecond))
            / Math.pow(poseEstimate.tagCount, 2);
    double generatedThetaDev = LimelightConstants.PoseEstimationConstants.baseVisionThetaDeviaition;

    Logger.recordOutput(
        "/Deviation Determinants/Average Tag Distance (m)", poseEstimate.avgTagDist);
    Logger.recordOutput(
        "/Deviation Determinants/Y Velocity (m-s)", drivetrainSpeeds.vyMetersPerSecond);
    Logger.recordOutput(
        "/Deviation Determinants/X Velocity (m-s)", drivetrainSpeeds.vxMetersPerSecond);
    Logger.recordOutput(
        "/Deviation Determinants/Angular Velocity (rad-s)", drivetrainSpeeds.omegaRadiansPerSecond);
    Logger.recordOutput("/Deviation Determinants/Tag Count", poseEstimate.tagCount);
    Logger.recordOutput(
        "/Deviation Determinants/Average Tag Ambiguity",
        Arrays.stream(poseEstimate.rawFiducials)
                .mapToDouble((tag) -> tag.ambiguity)
                .sum()
            / poseEstimate.rawFiducials.length);
    Logger.recordOutput(
        "/Deviation Determinants/Generated Deviations",
        new double[] {generatedXDev, generatedYDev, generatedThetaDev});

    return VecBuilder.fill(generatedXDev, generatedYDev, generatedThetaDev);
  }

  public double getDistanceToTagMeters(int tagID) {
    var tagPose = fieldMap.getTagPose(tagID).get().getTranslation();
    var robotPose3d = new Pose3d(this.getEstimatedPosition()).getTranslation();

    return tagPose.getDistance(robotPose3d);
  }

  public double getBlueRelativeHeadingDegrees() {
    var currentHeadingRotations = gyroAngle.get().getRotations();
    if (!this.isBlueAlliance()) currentHeadingRotations += 0.5;
    return Units.Rotations.of(currentHeadingRotations).in(Units.Degrees);
  }

  @Override
  public void resetRotation(Rotation2d rotation2d) {
    if (!this.isBlueAlliance())
      super.resetRotation(rotation2d.rotateBy(new Rotation2d(Math.toRadians(180))));
    else super.resetRotation(rotation2d);
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().map((optional) -> optional).orElse(Alliance.Blue)
        == Alliance.Blue;
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Estimated Swerve Pose", this.getEstimatedPosition());
    NameConstants.LimelightKeys.forEach(
        (limelightName) ->
            LimelightIO.setRobotOrientation(limelightName, this.getBlueRelativeHeadingDegrees()));

    if (currentTimeSeconds == null || gyroAngle == null || wheelPositions == null) {
      return;
    }

    this.updateWithTime(currentTimeSeconds.get(), gyroAngle.get(), wheelPositions.get());

    if (poseEstimateSuppliers.size() < 1) {
      return;
    }

    // Update the queue with the newest valid pose estimates
    poseEstimateSuppliers.stream()
        .map((estimator) -> estimator.get()) // convert to pose estimations
        .filter((estimate) -> this.isValidPoseEstimate(estimate)) // remove invalid pose estimations
        .forEach(
            (validEstimate) ->
                poseEstimateQueue.add(validEstimate)); // add valid estimates to the queue
    Pose2d[] acceptedPoseUpdates =
        poseEstimateQueue.stream()
            .map((estimate) -> estimate.getSecond().pose)
            .toArray(Pose2d[]::new);

    Logger.recordOutput("Accepted Pose Updates", acceptedPoseUpdates);

    while (!poseEstimateQueue.isEmpty()) {
      var estimate = poseEstimateQueue.poll().getSecond();

      if (estimate == null) continue;

      if (this.useDynamicVisionDeviations && robotSpeeds != null) {
        this.setVisionMeasurementStdDevs(
            this.generateDynamicVisionDeviations(estimate, robotSpeeds.get()));
      }

      this.addVisionMeasurement(
          estimate.pose, Units.Seconds.of(estimate.timestampSeconds).in(Units.Seconds));
    }
  }
}
