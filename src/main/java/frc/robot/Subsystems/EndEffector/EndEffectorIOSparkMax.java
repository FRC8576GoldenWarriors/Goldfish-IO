package frc.robot.Subsystems.EndEffector;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.drivers.WarriorSparkMax;

public class EndEffectorIOSparkMax implements EndEffectorIO {
  private WarriorSparkMax motor;
  private DigitalInput algaeInput;
  private DigitalInput coralInput;

  public EndEffectorIOSparkMax() {
    motor =
        new WarriorSparkMax(
            EndEffectorConstants.HardwareConstants.pincherID,
            MotorType.kBrushless,
            EndEffectorConstants.HardwareConstants.motorIsInverted,
            IdleMode.kBrake,
            60);
    algaeInput = new DigitalInput(EndEffectorConstants.HardwareConstants.algaeDigiSensorID);
    coralInput = new DigitalInput(EndEffectorConstants.HardwareConstants.coralLeftDigiSensorID);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.algaeInput = !algaeInput.get();
    inputs.coralInput = !coralInput.get();
    inputs.motorRunning = motor.get() != 0;
    inputs.current = motor.getOutputCurrent();
    inputs.voltage = motor.getAppliedOutput();
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
