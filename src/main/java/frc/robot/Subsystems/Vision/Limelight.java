package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {

  private List<Pair<LimelightIO, VisionIOInputsAutoLogged>> limelightInputAndOutput =
      new ArrayList<>();

  public Limelight(LimelightIO... limelightIOs) {
    for (int i = 0; i < limelightIOs.length; i++) {
      limelightInputAndOutput.add(Pair.of(limelightIOs[i], new VisionIOInputsAutoLogged()));
    }
  }

  private VisionIOInputsAutoLogged getInputsFromLimelightName(String limelightName) {
    return limelightInputAndOutput.stream()
        .filter(cameraPair -> cameraPair.getFirst().getLimelightName().equals(limelightName))
        .findFirst()
        .get()
        .getSecond();
  }

  public double getCurrentTagHeading(String limelightName) {

    boolean hasTargets = this.hasTargets(limelightName);
    int tagID = this.getTagID(limelightName);
    
    if(!hasTargets && tagID == -1) return 0;
    
    return LimelightConstants.PhysicalConstants.tagMap.get(tagID);
    
  }

  public boolean hasTargets(String limelightName) {
    return this.getInputsFromLimelightName(limelightName).hasTargets;
  }

  public int getTagID(String limelightName) {
    return this.getInputsFromLimelightName(limelightName).tagId;
  }

  public boolean isConnected(String limelightName) {
    return this.getInputsFromLimelightName(limelightName).connected;
  }

  public double getDistanceToTag(String limelightName, boolean isMegaTag2) {
    return isMegaTag2
        ? this.getInputsFromLimelightName(limelightName).megaTag2distanceToTagMeters
        : this.getInputsFromLimelightName(limelightName).megaTag1distanceToTagMeters;
  }

  public double getYaw(String limelightName) {
    return this.getInputsFromLimelightName(limelightName).yaw;
  }

  public double getPitch(String limelightName) {
    return this.getInputsFromLimelightName(limelightName).pitch;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < limelightInputAndOutput.size(); i++) {

      var curPair = limelightInputAndOutput.get(i);

      var io = curPair.getFirst();
      var input = curPair.getSecond();

      io.updateInputs(input);
      Logger.processInputs(io.getLimelightName(), input);
    }
  }
}
