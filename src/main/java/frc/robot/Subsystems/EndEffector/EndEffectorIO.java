package frc.robot.Subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  default void updateInputs(EndEffectorIOInputs inputs) {}

  @AutoLog
  public class EndEffectorIOInputs {
    public boolean algaeInput = false;
    public boolean coralInput = false;
    public boolean motorRunning = false;
    public double voltage = 0.0;
    public double current = 0.0;
  }

  default void setSpeed(double speed) {}
}
