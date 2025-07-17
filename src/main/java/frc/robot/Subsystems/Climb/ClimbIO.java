package frc.robot.Subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  default void updateInputs(ClimbIOInputs inputs) {}

  @AutoLog
  public class ClimbIOInputs {
    public double voltage = 0.0;
    public double current = 0.0;
    public double climbPosition = 0.0;
    public double relativeEncoderPosition = 0.0;
  }

  default void setVoltage(double voltage) {}

  default void setSpeed(double speed) {}

  default void stop() {}
}
