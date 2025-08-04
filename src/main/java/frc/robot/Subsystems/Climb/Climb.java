// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorBangBangController;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private ClimbIO io;

  private WarriorBangBangController bangBangController;

  public enum climbStates {
    VoltageControl,
    ClimbUp,
    ClimbDown,
    Slack,
    Idle,
    VoltSlack
  }

  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private climbStates climbAngle = climbStates.Idle;
  private double motorOutput;

  public Climb(ClimbIO io) {
    this.io = io;
    bangBangController = new WarriorBangBangController(0.005, 0.24);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    if (DriverStation.isEnabled()) {
      switch (climbAngle) {
        case VoltageControl:
          if (RobotContainer.driverController.povUp().getAsBoolean()) {
            motorOutput = 1.0;
            break;
          } else if (RobotContainer.driverController.povDown().getAsBoolean()) {
            motorOutput = -1.0;
            break;
          } else {
            climbAngle = climbStates.Idle;
            break;
          }
        case ClimbUp:
          motorOutput =
              bangBangController.calculate(
                  getPosition(), ClimbConstants.ControlConstants.climberUpPosition);
          break;
        case ClimbDown:
          motorOutput = bangBangController.calculate(getPosition(), 0.07);
          break;
        case Slack:
          motorOutput = bangBangController.calculate(getPosition(), 0.014);
          break;
        case Idle:
          motorOutput = 0.0;
          break;
        case VoltSlack:
          motorOutput = -1.0;

        default:
          break;
      }
    } else {
      climbAngle = climbStates.Idle;
    }
    io.setSpeed(motorOutput);

    Logger.recordOutput("Climb/Wanted Angle", climbAngle);
    // This method will be called once per scheduler run
  }

  public void setClimbAngle(climbStates climbAngle) {
    this.climbAngle = climbAngle;
  }

  public double getPosition() {
    return inputs.climbPosition;
  }

  public double getVoltage() {
    return inputs.voltage;
  }

  public double getCurrent() {
    return inputs.current;
  }

  public double getRelativePosition() {
    return inputs.relativeEncoderPosition;
  }

  public boolean nearSetpoint() {
    return bangBangController.nearSetpoint();
  }
}
