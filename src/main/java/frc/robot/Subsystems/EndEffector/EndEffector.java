// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  private EndEffectorIO io;

  public enum EndEffectorStates {
    Rest,
    AlgaeIntake,
    CoralIntake,
    CoralOut,
    L3Out
  }

  private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private EndEffectorStates wantedState = EndEffectorStates.Rest;
  private double wantedSpeed;

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);
    switch (wantedState) {
      case Rest:
        wantedSpeed = 0.0;
        break;
      case AlgaeIntake:
        wantedSpeed = EndEffectorConstants.ControlConstants.pincherAlgaeSpeed;
        break;
      case CoralIntake:
        wantedSpeed = EndEffectorConstants.ControlConstants.pincherCoralInSpeed;
        break;
      case CoralOut:
        wantedSpeed = EndEffectorConstants.ControlConstants.pincherCoralOutSpeed;
        break;
      case L3Out:
        wantedSpeed = EndEffectorConstants.ControlConstants.pincherCoralL3OutSpeed;
        break;
      default:
        wantedSpeed = 0.0;
        break;
    }
    Logger.recordOutput("Wanted State", wantedState);
    Logger.recordOutput("Wanted Speed", wantedSpeed);
    io.setSpeed(wantedSpeed);
    // This method will be called once per scheduler run
  }

  public void setWantedState(EndEffectorStates state) {
    wantedState = state;
  }
  public boolean getAlgaeInput(){
    return inputs.algaeInput;
  }
  public EndEffectorStates getState(){
    return wantedState;
  }
}
