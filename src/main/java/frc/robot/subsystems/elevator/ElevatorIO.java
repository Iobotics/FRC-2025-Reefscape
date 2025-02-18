package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean motor1Connected = true;
    public boolean motor2Connected = true;
    public boolean motor3Connected = true;
    public boolean motor4Connected = true;
    public double[] positionRotations = new double[] {};
    public double[] positionMeters = new double[] {};
    public double[] velocityMeters = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] appliedVolts = new double[] {};
    public double[] setpointRotations = new double[] {};
    public boolean atGoal = false;
  }

  default boolean atGoal() {
    return false;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void stop() {}

  default void runVolts(double volts) {}

  default void runSetpoint(double setpointMeters, double feedforward) {}

  default void runSetpointMotionMagic(double setpointMeters, double feedforward) {}

  default void runCurrent(double amps) {}

  default void setMotionMagicConstraints(double velocity, double acceleration, double jerk) {}

  default void setPID(double p, double i, double d, double v, double s, double a, double g) {}
}
