package frc.robot.Subsystems.Arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.drivers.WarriorSparkMax;

public class ArmIOSparkMax implements ArmIO {
  private WarriorSparkMax motor;
  private DutyCycleEncoder absEncoder;

  public ArmIOSparkMax() {
    motor =
        new WarriorSparkMax(
            ArmConstants.HardwareConstants.armMotorID,
            MotorType.kBrushless,
            ArmConstants.ControlConstants.motorIsInverted,
            IdleMode.kCoast,
            40);

    absEncoder =
        new DutyCycleEncoder(
            ArmConstants.HardwareConstants.armEncoderDIO,
            1.0,
            ArmConstants.ControlConstants.armEncoderOffset);
    absEncoder.setInverted(ArmConstants.ControlConstants.armEncoderIsInverted);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.voltage = motor.getAppliedOutput();
    inputs.current = motor.getOutputCurrent();
    inputs.encoderValue = absEncoder.get();
    inputs.velocity = motor.getEncoder().getVelocity();
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
