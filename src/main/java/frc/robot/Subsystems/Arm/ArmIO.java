package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  default void updateInputs(ArmIOInputs inputs) {}

  @AutoLog
  class ArmIOInputs {
    public double voltage = 0.0;
    public double current = 0.0;
    public double encoderValue = 0.0;
    public double velocity = 0.0;
  }

  default void setVoltage(double voltage) {}

  default void setSpeed(double speed) {}
}
