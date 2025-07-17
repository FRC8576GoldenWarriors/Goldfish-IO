package frc.robot.Subsystems.Climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.drivers.WarriorSparkMax;

public class ClimbIOSparkMax implements ClimbIO {
  private WarriorSparkMax motor;
  private DutyCycleEncoder absEncoder;

  public ClimbIOSparkMax() {
    motor =
        new WarriorSparkMax(
            ClimbConstants.HardwareConstants.motorID,
            MotorType.kBrushless,
            ClimbConstants.HardwareConstants.motorIsInverted,
            IdleMode.kBrake,
            80);

    absEncoder =
        new DutyCycleEncoder(
            ClimbConstants.HardwareConstants.climberEncoderDIO,
            1.0,
            ClimbConstants.ControlConstants.climberEncoderOffset);
    absEncoder.setInverted(ClimbConstants.HardwareConstants.climberEncoderIsInverted);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.climbPosition = absEncoder.get();
    inputs.current = motor.getOutputCurrent();
    inputs.voltage = motor.getAppliedOutput();
    inputs.relativeEncoderPosition = motor.getEncoder().getPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
