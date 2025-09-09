// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.SwerveDrive.SwerveConstants;
import frc.robot.Subsystems.Vision.Limelight.Limelight;
import frc.robot.Subsystems.Vision.Limelight.LimelightConstants;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionReefAlign extends Command {

  private final Drivetrain drivetrain;
  private final Limelight limelight;
  private final String limelightName = LimelightConstants.NameConstants.REEF_NETWORKTABLE_KEY;

  private final ProfiledPIDController rotationPID;
  private final PIDController forwardPID;
  private final PIDController strafePID;

  private double driveOutput;
  private double rotationOutput;
  private double strafeOutput;

  private ReefAlignState wantedAlignState;

  private double wantedStrafeDistance = 0;

  public enum ReefAlignState {
    LeftSide,
    RightSide,
    Middle
  }

  public VisionReefAlign(
      Drivetrain requirements, Limelight limelight, ReefAlignState wantedAlignState) {

    this.drivetrain = requirements;
    this.limelight = limelight;
    this.wantedAlignState = wantedAlignState;

    rotationPID =
        new ProfiledPIDController(
            LimelightConstants.PIDConstants.rotationkP,
            LimelightConstants.PIDConstants.rotationkI,
            LimelightConstants.PIDConstants.rotationkD,
            new Constraints(
                SwerveConstants.DRIVETRAIN_MAX_ANGULAR_SPEED,
                SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION));
    rotationPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_ANGLE_ERROR);
    rotationPID.enableContinuousInput(-180, 180);

    forwardPID =
        new PIDController(
            LimelightConstants.PIDConstants.forwardkP,
            LimelightConstants.PIDConstants.forwardkI,
            LimelightConstants.PIDConstants.forwardkD);
    forwardPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_DISTANCE_ERROR);

    strafePID =
        new PIDController(
            LimelightConstants.PIDConstants.strafekP,
            LimelightConstants.PIDConstants.strafekI,
            LimelightConstants.PIDConstants.strafekD);
    strafePID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_STRAFE_ERROR);

    // switch (wantedAlignState) {
    //   case Middle:
    //     wantedStrafeDistance = 0.0;
    //   case LeftSide:
    //     wantedStrafeDistance = LimelightConstants.PhysicalConstants.LEFT_STICK_OFFSET;
    //   case RightSide:
    //     wantedStrafeDistance = LimelightConstants.PhysicalConstants.RIGHT_STICK_OFFSET;
    // }
    wantedStrafeDistance = 0.0;
    addRequirements(requirements, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationPID.reset(drivetrain.getHeading(), drivetrain.getRotationVelocity());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!limelight.hasTargets(limelightName)
        || limelight.getTagID(limelightName) == 13
        || limelight.getTagID(limelightName) == 12
        || limelight.getTagID(limelightName) == 1
        || limelight.getTagID(limelightName) == 2) return;
    double distanceToTagMeters = limelight.getDistanceToTag(limelightName, true);
    double verticalAngle = limelight.getPitch(limelightName);
    double currentHeading = drivetrain.getHeading();
    double cameraPitchDegrees =
        Units.radiansToDegrees(
            LimelightConstants.PositionalConstants.REEF_LIMELIGHT_LOCATION.getRotation().getY());
    double distanceToWall =
        distanceToTagMeters * Math.cos(Units.degreesToRadians(cameraPitchDegrees + verticalAngle));
    double horizontalAngle = limelight.getYaw(limelightName);

    double strafeDistance = distanceToWall * Math.tan(Units.degreesToRadians(horizontalAngle));
    strafeOutput = strafePID.calculate(strafeDistance, wantedStrafeDistance);
    rotationOutput =
        rotationPID.calculate(currentHeading, limelight.getCurrentTagHeading(limelightName));
    driveOutput =
        forwardPID.calculate(
            distanceToWall, LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_REEF);

    drivetrain.drive(new Translation2d(driveOutput, -strafeOutput), rotationOutput, false, true);
    Logger.recordOutput("Reef Align/Forward PID", driveOutput);
    Logger.recordOutput("Reef Align/Strafe PID", -strafeOutput);
    Logger.recordOutput("Reef Align/Rotation PID", rotationOutput);
    Logger.recordOutput("Reef Align/Forward PID Setpoint", forwardPID.getSetpoint());
    Logger.recordOutput("Reef Align/Strafe PID Setpoint", strafePID.getSetpoint());
    Logger.recordOutput("Reef Align/Rotation PID Setpoint", rotationPID.getSetpoint().position);
    Logger.recordOutput("Reef Align/Current Strafe Position", strafeDistance);
    Logger.recordOutput("Reef Align/Distance To Tag Meters", distanceToWall);
    Logger.recordOutput("Reef Align/Forward PID At Setpoint", forwardPID.atSetpoint());
    Logger.recordOutput("Reef Align/Strafe PID At Setpoint", strafePID.atSetpoint());
    Logger.recordOutput("Reef Align/Rotation PID At Goal", rotationPID.atGoal());
    Logger.recordOutput(
        "Reef Align/Command Finished",
        forwardPID.atSetpoint() && strafePID.atSetpoint() && rotationPID.atGoal());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new Translation2d(), 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return forwardPID.atSetpoint() && strafePID.atSetpoint() && rotationPID.atGoal();
  }
}
