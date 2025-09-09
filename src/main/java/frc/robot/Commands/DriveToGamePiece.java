// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.Vision.PhotonVision.PhotonVision;
import frc.robot.Subsystems.Vision.PhotonVision.PhotonVisionConstants;
import java.util.Arrays;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToGamePiece extends Command {

  Drivetrain drivetrain;
  PhotonVision photonVision;
  int gamePieceID;

  public enum GamePiece {
    ALGAE,
    CORAL
  }

  public DriveToGamePiece(Drivetrain drivetrain, PhotonVision photonVision, GamePiece gamePiece) {

    this.drivetrain = drivetrain;
    this.photonVision = photonVision;

    switch (gamePiece) {
      case ALGAE:
        this.gamePieceID = PhotonVisionConstants.PhysicalConstants.ALGAE_ID;
        break;

      case CORAL:
        this.gamePieceID = PhotonVisionConstants.PhysicalConstants.CORAL_ID;
        break;
    }

    addRequirements(drivetrain, photonVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var leftTargets =
        photonVision.getListOfTargetPoses(PhotonVisionConstants.NameConstants.LEFT_CAMERA);
    var leftIDs = photonVision.getListOfObjectIDs(PhotonVisionConstants.NameConstants.LEFT_CAMERA);

    var rightTargets =
        photonVision.getListOfTargetPoses(PhotonVisionConstants.NameConstants.RIGHT_CAMERA);
    var rightIDs =
        photonVision.getListOfObjectIDs(PhotonVisionConstants.NameConstants.RIGHT_CAMERA);

    if (leftTargets.size() == 0 && rightTargets.size() == 0) return;

    Pose2d bestPiecePoseFromTheLeft = leftTargets.get(leftIDs.indexOf(gamePieceID));
    Pose2d bestPiecePoseFromTheRight = rightTargets.get(rightIDs.indexOf(gamePieceID));

    Pose2d overallBestPose =
        drivetrain
            .getPose2d()
            .nearest(Arrays.asList(bestPiecePoseFromTheLeft, bestPiecePoseFromTheRight));

    drivetrain.drive(
        overallBestPose.getTranslation(), overallBestPose.getRotation().getDegrees(), false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
