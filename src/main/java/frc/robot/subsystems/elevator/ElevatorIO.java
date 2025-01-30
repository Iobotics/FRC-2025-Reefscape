package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;
    public double[] positionRotations = new double[] {};
    public double[] positionMeters = new double[] {};
    public double[] velocityMeters = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] appliedVolts = new double[] {};
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void stop() {}

  default void runVolts(double volts) {}

  default void runSetpoint(double setpointMeters, double feedforward) {}

  default void runCurrent(double amps) {}

  default void setPID(double p, double i, double d) {}
}
