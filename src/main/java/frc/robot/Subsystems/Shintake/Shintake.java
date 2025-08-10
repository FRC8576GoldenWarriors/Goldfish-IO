// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shintake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shintake extends SubsystemBase {
  /** Creates a new Shintake. */
  private ShintakeIO io;

  public enum ShintakeStates {
    Rest,
    AlgaeIntake,
    // AlgaeIntakeAuto,
    Transfer,
    Shoot,
    AlgaeOuttake
  }

  private ShintakeIOInputsAutoLogged inputs = new ShintakeIOInputsAutoLogged();
  private double bottomRPM, upperRPM;
  private ShintakeStates wantedState = ShintakeStates.Rest;
  private double bottomSpeed, upperSpeed;

  public Shintake(ShintakeIO io) {
    bottomRPM = 0.0;
    upperRPM = 0.0;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shintake", inputs);

    if (DriverStation.isEnabled()) {
      switch (wantedState) {
        case Rest:
          bottomRPM = 0.0;
          upperRPM = 0.0;
          io.setRollersRPM(bottomRPM, upperRPM);
          break;
        case AlgaeIntake:
          bottomSpeed = -0.5;
          upperSpeed = -0.5;
          io.setRollersSpeed(bottomSpeed, upperSpeed);
          // case AlgaeIntakeAuto:
          //   bottomSpeed = -0.8;
          //   upperSpeed = -0.8;
          //   io.setRollersSpeed(bottomSpeed, upperSpeed);
          //   break;
        case Transfer:
          bottomSpeed = -0.3;
          upperSpeed = -0.3;
          io.setRollersSpeed(bottomSpeed, upperSpeed);
          break;
        case AlgaeOuttake:
          bottomSpeed = -1.0;
          upperSpeed = -1.0;
          io.setRollersSpeed(bottomSpeed, upperSpeed);
          break;
        case Shoot:
          bottomRPM = ShintakeConstants.ControlConstants.shootBottomRPM;
          upperRPM = ShintakeConstants.ControlConstants.shootUpperRPM;
          io.setRollersRPM(bottomRPM, upperRPM);
          break;
          // default:
          //   bottomRPM = 0.0;
          //   upperRPM = 0.0;
          //   io.setRollersRPM(bottomRPM, upperRPM);
          //   break;
      }
    } else {
      wantedState = ShintakeStates.Rest;
    }

    Logger.recordOutput("Shintake/Wanted State", wantedState);
    Logger.recordOutput("Shintake/Is Revved", shootersRevved());
    Logger.recordOutput("Shintake/Bottom Roller RPM", getShooterRMPs()[0]);
    Logger.recordOutput("Shintake/Top Roller RPM", getShooterRMPs()[1]);
    // This method will be called once per scheduler run
  }

  public void setWantedState(ShintakeStates state) {
    wantedState = state;
  }

  public double[] getShooterRMPs() {
    return new double[] {inputs.lowerRollerRPM, inputs.upperRollerRPM};
  }

  public boolean getAlgaeDetected() {
    return inputs.algaeDetected;
  }

  public double getLowerCurrent() {
    return inputs.lowerRollerMotorCurrent;
  }

  public boolean shootersRevved() {
    return Math.abs(getShooterRMPs()[0] - ShintakeConstants.ControlConstants.shootBottomRPM) < 50;
  }

  public ShintakeStates getState() {
    return wantedState;
  }
}
