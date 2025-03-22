package frc.robot.subsystems.climb.pivot;

import org.littletonrobotics.junction.Logger;

public class Pivot {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public Pivot(PivotIO io) {
    this.io = io;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb/Pivot", inputs);
  }
}
