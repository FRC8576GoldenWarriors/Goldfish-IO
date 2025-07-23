package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {

  List<LimelightIO> limelightIOs = new ArrayList<>();
  List<VisionIOInputsAutoLogged> limelightInputs = new ArrayList<>();

  public Limelight(LimelightIO... limelightIOs) {
    for (int i = 0; i < limelightIOs.length; i++) {
      this.limelightIOs.add(limelightIOs[i]);
      this.limelightInputs.add(new VisionIOInputsAutoLogged());
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < limelightIOs.size(); i++) {
      var io = limelightIOs.get(i);
      var input = limelightInputs.get(i);

      io.updateInputs(input);
      Logger.processInputs(io.getLimelightName(), input);
    }
  }
}
