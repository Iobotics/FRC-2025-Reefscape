package frc.robot.subsystems.climb.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean motorConnected = false;
    public double positionRadians = 0.0;
    public double appliedVoltage = 0.0;
    public double current = 0.0;
    public double temperature = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
