package frc.robot.Subsystems.Shintake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.drivers.WarriorSparkMax;

public class ShintakeIOSparkMax implements ShintakeIO {
  private WarriorSparkMax lowerRollerMotor;
  private WarriorSparkMax upperRollerMotor;

  private DigitalInput lowerRollerDigitalInput;

  public ShintakeIOSparkMax() {
    lowerRollerMotor =
        new WarriorSparkMax(
            ShintakeConstants.HardwareConstants.rollerMotorLowID,
            MotorType.kBrushless,
            ShintakeConstants.HardwareConstants.rollerMotorLowIsInverted,
            IdleMode.kCoast,
            60);

    upperRollerMotor =
        new WarriorSparkMax(
            ShintakeConstants.HardwareConstants.rollerMotorHighID,
            MotorType.kBrushless,
            ShintakeConstants.HardwareConstants.rollerMotorHighIsInverted,
            IdleMode.kCoast,
            60);

    lowerRollerDigitalInput =
        new DigitalInput(ShintakeConstants.HardwareConstants.lowerRollerDigitalInputDIO);

    // DO NOT CHANGE WITHOUT OFFICER/DEPUTY SUPERVISION - Kevin
    lowerRollerMotor.setkF(1 / 5800.0); // 1/4730.0
    lowerRollerMotor.setkP(0.0003); // 0.0005
    lowerRollerMotor.setkI(0.0);
    lowerRollerMotor.setkD(0.000003);
    lowerRollerMotor.setMaxMotion(5600, 12000);

    upperRollerMotor.setkF(1 / 6100.0);
    upperRollerMotor.setkP(0.0003);
    upperRollerMotor.setkI(0.0);
    upperRollerMotor.setkD(0.000003);
    upperRollerMotor.setMaxMotion(5600, 12000);
  }

  @Override
  public void setRollersRPM(double lowerRPM, double upperRPM) {
    lowerRollerMotor
        .getClosedLoopController()
        .setReference(lowerRPM, ControlType.kMAXMotionVelocityControl);
    upperRollerMotor
        .getClosedLoopController()
        .setReference(upperRPM, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void setRollersSpeed(double lowerSpeed, double upperSpeed) {
    lowerRollerMotor.set(lowerSpeed);
    upperRollerMotor.set(upperSpeed);
  }

  @Override
  public void updateInputs(ShintakeIOInputs inputs) {
    inputs.lowerRollerMotorVoltage = lowerRollerMotor.getAppliedOutput();
    inputs.lowerRollerMotorCurrent = lowerRollerMotor.getOutputCurrent();
    inputs.lowerRollerRPM = lowerRollerMotor.getEncoder().getVelocity();

    inputs.upperRollerMotorVoltage = upperRollerMotor.getAppliedOutput();
    inputs.upperRollerMotorCurrent = upperRollerMotor.getOutputCurrent();
    inputs.upperRollerRPM = upperRollerMotor.getEncoder().getVelocity();

    inputs.algaeDetected = !lowerRollerDigitalInput.get();
  }
}
