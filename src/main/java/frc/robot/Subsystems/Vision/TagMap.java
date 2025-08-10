package frc.robot.Subsystems.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SwerveDrive.SwerveConstants;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

public class TagMap {

  AprilTagFieldLayout fieldLayout;

  List<AprilTag> tags;

  public enum Face {
    FrontSide,
    LeftSide,
    BackSide,
    RightSide
  }

  public enum Tags {
    ALL,
    REEF,
    BARGE,
    CORAL,
    PROCCESSOR
  }

  public TagMap(AprilTagFields field, Tags tagEnum) {
    fieldLayout = AprilTagFieldLayout.loadField(field);
    tags = fieldLayout.getTags();

    switch (tagEnum) {
      case ALL:
        break;
      case REEF:
        tags =
            tags.stream()
                .filter(
                    a -> {
                      return AprilTagConstants.GameObjectIDConstants.REEF_TAG_IDS.contains(
                          Integer.valueOf(a.ID));
                    })
                .toList();
        break;
      case BARGE:
        tags =
            tags.stream()
                .filter(
                    a -> {
                      return AprilTagConstants.GameObjectIDConstants.BARGE_TAG_IDS.contains(
                          Integer.valueOf(a.ID));
                    })
                .toList();
        break;
      case CORAL:
        tags =
            tags.stream()
                .filter(
                    a -> {
                      return AprilTagConstants.GameObjectIDConstants.CORAL_STATION_TAG_IDS.contains(
                          Integer.valueOf(a.ID));
                    })
                .toList();
        break;
      case PROCCESSOR:
        tags =
            tags.stream()
                .filter(
                    a -> {
                      return AprilTagConstants.GameObjectIDConstants.PROCESSOR_TAG_IDS.contains(
                          Integer.valueOf(a.ID));
                    })
                .toList();
        break;
    }
  }

  public TagMap(AprilTagFields field, List<Integer> tagIDs) {
    fieldLayout = AprilTagFieldLayout.loadField(field);
    tags =
        tagIDs.stream()
            .map(a -> new AprilTag(a.intValue(), fieldLayout.getTagPose(a).get()))
            .toList();
  }

  public Pose3d getTagPose3d(int tagID) {

    return (tagID > -1) ? fieldLayout.getTagPose(tagID).get() : null;
  }

  public Translation3d getTagTranslation3d(int tagID) {
    return this.getTagPose3d(tagID).getTranslation();
  }

  public Translation2d getTagTranslation2d(int tagID) {
    return this.getTagPose3d(tagID).getTranslation().toTranslation2d();
  }

  public Rotation3d getTagRotation3d(int tagID) {
    return this.getTagPose3d(tagID).getRotation();
  }

  public Rotation2d getTagRotation2d(int tagID) {
    return this.getTagPose3d(tagID).getRotation().toRotation2d();
  }

  // clamped [-180, 180]
  public double getAlignRotationInDegrees(int tagID, Face faceSide) {

    double rotOffset = 0;

    switch (faceSide) {
      case FrontSide:
        rotOffset = 180;
        break;
      case LeftSide:
        rotOffset = -90;
        break;
      case BackSide:
        rotOffset = 0;
        break;
      case RightSide:
        rotOffset = 90;
        break;
    }

    return Math.IEEEremainder(
        Units.radiansToDegrees(this.getTagRotation3d(tagID).getZ()) - rotOffset, 360);
  }

  public Transform2d getRobotTransformToTag(int tagID, Pose2d robotPose) {
    return new Transform2d(robotPose, this.getTagPose3d(tagID).toPose2d());
  }

  public int getTagIDClosestToRobotPose(Pose3d robotPose) {
    return tags.stream()
        .min(
            Comparator.comparingDouble(
                tag ->
                    robotPose
                        .getTranslation()
                        .toTranslation2d()
                        .getDistance(tag.pose.getTranslation().toTranslation2d())))
        .map(tag -> tag.ID)
        .orElse(-1);
  }

  public int getTagIDClosestToRobotPose(Pose2d robotPose) {
    return tags.stream()
        .min(
            Comparator.comparingDouble(
                tag ->
                    robotPose
                        .getTranslation()
                        .getDistance(tag.pose.getTranslation().toTranslation2d())))
        .map(tag -> tag.ID)
        .orElse(-1);
  }

  public Pose2d getTagPoseToMoveTo(int tagID, double distFromFaceOffset, Face faceSide) {
    Pose2d tagPose = this.getTagPose3d(tagID).toPose2d();

    double robotSizeOffset = 0;

    switch (faceSide) {
      case FrontSide:
        robotSizeOffset = SwerveConstants.TRACK_WIDTH / 2;
        break;

      case LeftSide:
        robotSizeOffset = SwerveConstants.WHEEL_BASE / 2;
        break;

      case BackSide:
        robotSizeOffset = SwerveConstants.TRACK_WIDTH / 2;
        break;

      case RightSide:
        robotSizeOffset = SwerveConstants.WHEEL_BASE / 2;
        break;

      default:
        robotSizeOffset = 0;
        break;
    }

    Rotation2d poseRotation2d =
        new Rotation2d(Units.degreesToRadians(this.getAlignRotationInDegrees(tagID, faceSide)));

    double xCoord =
        (tagPose.getX()
            + (distFromFaceOffset + robotSizeOffset)
                * Math.cos(tagPose.getRotation().getRadians()));
    double yCoord =
        (tagPose.getY()
            + (distFromFaceOffset + robotSizeOffset)
                * Math.sin(tagPose.getRotation().getRadians()));

    Translation2d poseTranslation2d = new Translation2d(xCoord, yCoord);

    return new Pose2d(poseTranslation2d, poseRotation2d);
  }

  public Pose2d getClosestTagPoseToMoveTo(
      double distFromFaceOffset, Face faceSide, Pose2d robotPose) {
    return this.getTagPoseToMoveTo(
        this.getTagIDClosestToRobotPose(robotPose), distFromFaceOffset, faceSide);
  }

  public Command getPathFindCommand(Subsystem... requirements) {
    return new DeferredCommand(
        () ->
            AutoBuilder.pathfindToPose(
                this.getClosestTagPoseToMoveTo(
                    0.25, Face.FrontSide, RobotContainer.m_Drivetrain.getPose2d()),
                PathFinderConstants.constraints),
        Set.of(requirements));
  }

  public Command getPathFindCommand(double faceOffset, Face faceSide, Subsystem... requirements) {
    return new DeferredCommand(
        () ->
            AutoBuilder.pathfindToPose(
                this.getClosestTagPoseToMoveTo(
                    faceOffset, faceSide, RobotContainer.m_Drivetrain.getPose2d()),
                PathFinderConstants.constraints),
        Set.of(requirements));
  }

  public Command getPathFindCommand(int tagID, Subsystem... requirements) {
    return new DeferredCommand(
        () ->
            AutoBuilder.pathfindToPose(
                this.getTagPoseToMoveTo(tagID, 0.25, Face.FrontSide),
                PathFinderConstants.constraints),
        Set.of(requirements));
  }

  public Command getPathFindCommand(int tagID, double faceOffset, Subsystem... requirements) {
    return new DeferredCommand(
        () ->
            AutoBuilder.pathfindToPose(
                this.getTagPoseToMoveTo(tagID, faceOffset, Face.FrontSide),
                PathFinderConstants.constraints),
        Set.of(requirements));
  }

  public Command getPathFindCommand(
      int tagID, double faceOffset, Face faceSide, Subsystem... requirements) {
    return new DeferredCommand(
        () ->
            AutoBuilder.pathfindToPose(
                this.getTagPoseToMoveTo(tagID, faceOffset, faceSide),
                PathFinderConstants.constraints),
        Set.of(requirements));
  }
}
