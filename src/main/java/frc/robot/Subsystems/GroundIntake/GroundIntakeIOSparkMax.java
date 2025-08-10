package frc.robot.Subsystems.GroundIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.GroundIntake.GroundIntake.GroundIntakeStates;

public class GroundIntakeIOSparkMax implements GroundIntakeIO {
  private WarriorSparkMax pivotMotor;
  private WarriorSparkMax rollerMotor;
  private DutyCycleEncoder encoder;
  private DigitalInput algaeSensor;

  public GroundIntakeIOSparkMax() {
    pivotMotor =
        new WarriorSparkMax(
            GroundIntakeConstants.HardwareConstants.pivotMotorID,
            MotorType.kBrushless,
            GroundIntakeConstants.HardwareConstants.pivotMotorIsInverted,
            IdleMode.kBrake,
            30);
    rollerMotor =
        new WarriorSparkMax(
            GroundIntakeConstants.HardwareConstants.rollerMotorID,
            MotorType.kBrushless,
            GroundIntakeConstants.HardwareConstants.rollerMotorIsInverted,
            IdleMode.kBrake,
            22);

    encoder =
        new DutyCycleEncoder(
            GroundIntakeConstants.HardwareConstants.pivotEncoderDIO,
            GroundIntakeConstants.ControlConstants.pivotEncoderFullRange,
            GroundIntakeConstants.ControlConstants.pivotEncoderZero);

    encoder.setInverted(GroundIntakeConstants.ControlConstants.pivotEncoderIsInverted);

    algaeSensor = new DigitalInput(GroundIntakeConstants.HardwareConstants.digitalInputDIO);
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {
    inputs.algaeDetected =
        !algaeSensor.get()
            && (RobotContainer.m_GroundIntake.getState() != GroundIntakeStates.Rest
                && RobotContainer.m_GroundIntake.getState() != GroundIntakeStates.Idle);
    inputs.encoderConnected = encoder.isConnected();
    inputs.encoderPosition = encoder.get();
    inputs.pivotCurrent = pivotMotor.getOutputCurrent();
    inputs.pivotVoltage = pivotMotor.getAppliedOutput();
    inputs.rollerSpeed = rollerMotor.get();
    inputs.rollerCurrent = rollerMotor.getOutputCurrent();
    inputs.rollerVoltage = rollerMotor.getAppliedOutput();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerMotor.setVoltage(voltage);
  }

  @Override
  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }
}
