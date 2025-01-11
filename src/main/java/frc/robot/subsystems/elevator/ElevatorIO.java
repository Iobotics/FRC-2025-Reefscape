package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean motorConnected = true;
    public double positionMeters = 0.0;
    public double velocityMeters = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double appliedVolts = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void stop() {}

  default void runVolts(double volts) {}

  default void runSetpoint(double setpointMeters, double feedforward) {}

  default void setPID(double p, double i, double d) {}
}
