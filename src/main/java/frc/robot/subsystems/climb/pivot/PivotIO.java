package frc.robot.subsystems.climb.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean motorConnected = false;
    public double positionRadians = 0.0;
    public double appliedVoltage = 0.0;
    public double current = 0.0;
    public double temperature = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
