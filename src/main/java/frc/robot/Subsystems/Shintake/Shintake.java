// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shintake extends SubsystemBase {
  /** Creates a new Shintake. */
  private ShintakeIO io;

  private ShintakeIOInputsAutoLogged inputs = new ShintakeIOInputsAutoLogged();
  private double bottomRPM, upperRPM;

  public Shintake(ShintakeIO io) {
    bottomRPM = 0.0;
    upperRPM = 0.0;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shintake", inputs);

    io.setRollersRPM(bottomRPM, upperRPM);
    // This method will be called once per scheduler run
  }

  public void setWantedRPMs(double bottomRPM, double upperRPM) {
    this.bottomRPM = bottomRPM;
    this.upperRPM = upperRPM;
  }
}
