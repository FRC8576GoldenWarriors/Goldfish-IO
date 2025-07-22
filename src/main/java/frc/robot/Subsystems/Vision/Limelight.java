package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.Vision.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

  List< Pair<Limelight, LimelightIOAutoLogged> > limelights;

  public Limelight(Pair<LimelightIO, LimelightIOAutoLogged>... limelightIOs) {
    limelights = Arrays.asList(limelightIOs);
  }

  @Override
  public void periodic() {

  }
}