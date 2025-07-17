package frc.robot.Subsystems.GroundIntake;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
  default void updateInputs(GroundIntakeIOInputs inputs) {}

  @AutoLog
  public class GroundIntakeIOInputs {
    public double encoderPosition = 0.0;
    public boolean algaeDetected = false;
    public double rollerVoltage = 0.0;
    public double rollerCurrent = 0.0;
    public double pivotVoltage = 0.0;
    public double pivotCurrent = 0.0;
    public double rollerSpeed = 0.0;
    public boolean encoderConnected = false;
  }

  default void setPivotVoltage(double voltage) {}

  default void setPivotSpeed(double speed) {}

  default void setRollerVoltage(double voltage) {}

  default void setRollerSpeed(double speed) {}
}
