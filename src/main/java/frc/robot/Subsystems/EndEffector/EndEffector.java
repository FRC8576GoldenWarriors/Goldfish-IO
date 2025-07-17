// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  private EndEffectorIO io;

  private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private double wantedSpeed = 0;

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);
    if (wantedSpeed >= -1 || wantedSpeed <= 1) {
      io.setSpeed(wantedSpeed);
    }
    // This method will be called once per scheduler run
  }

  public void setWantedSpeed(double speed) {
    wantedSpeed = speed;
  }
}
