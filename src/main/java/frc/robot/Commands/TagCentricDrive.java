// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SwerveDrive.SwerveConstants;
import frc.robot.Subsystems.Vision.Limelight.LimelightConstants;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TagCentricDrive extends Command {

  private frc.robot.Subsystems.SwerveDrive.Drivetrain drivetrain = RobotContainer.m_Drivetrain;

  ProfiledPIDController rotationPID;
  double rotOutput;

  // Command for rotating to face the barge when driving near it; passive barge align.
  public TagCentricDrive() {

    rotationPID =
        new ProfiledPIDController(
            LimelightConstants.PIDConstants.ROTATION_KP,
            LimelightConstants.PIDConstants.ROTATION_KI,
            LimelightConstants.PIDConstants.ROTATION_KD,
            new Constraints(
                SwerveConstants.DRIVETRAIN_MAX_ANGULAR_SPEED,
                SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION));
    rotationPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_ANGLE_ERROR);
    rotationPID.enableContinuousInput(-180, 180);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    rotationPID.reset(drivetrain.getHeading(), drivetrain.getRotationVelocity());
  }

  @Override
  public void execute() {

    Pose2d robotPose = drivetrain.getPose();
    int closetBargeTagID = RobotContainer.m_TagMap.getTagIDClosestToBargeFromRobotPose(robotPose);
    Translation2d closetBargeTagTranslation =
        RobotContainer.m_TagMap.getTagTranslation2d(closetBargeTagID);

    double hypotenuseTranslationToTag =
        closetBargeTagTranslation.getDistance(robotPose.getTranslation());
    double xTranslationToTag = Math.abs(closetBargeTagTranslation.getX() - robotPose.getX());
    double yTranslationToTag = Math.abs(closetBargeTagTranslation.getY() - robotPose.getY());

    double angleToTag = Math.atan(xTranslationToTag / yTranslationToTag);

    Logger.recordOutput("X distance", xTranslationToTag);
    Logger.recordOutput("Y distance", yTranslationToTag);
    Logger.recordOutput("Theta angle", angleToTag);

    if (hypotenuseTranslationToTag < 2) {
      rotOutput =
          rotationPID.calculate(drivetrain.getHeading(), Units.degreesToRadians(angleToTag));
    } else {
      rotOutput =
          -RobotContainer.driverController.getRightX()
              * Math.abs(RobotContainer.driverController.getRightX())
              * SwerveConstants.DriverConstants.turnCoefficient;
    }

    Logger.recordOutput("Centric Rotation Output", rotOutput);

    RobotContainer.m_Drivetrain.swerveDrive(
        -RobotContainer.driverController.getLeftY()
            * Math.abs(RobotContainer.driverController.getLeftY())
            * SwerveConstants.DriverConstants.xCoefficient, // 2.25
        -RobotContainer.driverController.getLeftX()
            * Math.abs(RobotContainer.driverController.getLeftX())
            * SwerveConstants.DriverConstants.yCoefficient,
        rotOutput, // 2.25)// 1.75
        true, // !RobotContainer.driverController.getHID().getRawButton(XboxController.Button.kB.value)
        new Translation2d(),
        true);
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
