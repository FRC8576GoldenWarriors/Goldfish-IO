package frc.robot.Subsystems.Shintake;

import org.littletonrobotics.junction.AutoLog;

public interface ShintakeIO {
  default void updateInputs(ShintakeIOInputs inputs) {}

  @AutoLog
  public class ShintakeIOInputs {
    public double lowerRollerMotorVoltage = 0.0;
    public double lowerRollerMotorCurrent = 0.0;
    public double lowerRollerRPM = 0.0;

    public double upperRollerMotorVoltage = 0.0;
    public double upperRollerMotorCurrent = 0.0;
    public double upperRollerRPM = 0.0;

    public boolean algaeDetected = false;
  }

  default void setRollersRPM(double LowerRPM, double UpperRPM) {}

  default void setRollersSpeed(double LowerSpeed, double UpperSpeed) {}
}
