package frc.robot.subsystems.Funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double velocityRotsPerSec = 0.0;
  }

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void runVoltage(double volts) {}
}
