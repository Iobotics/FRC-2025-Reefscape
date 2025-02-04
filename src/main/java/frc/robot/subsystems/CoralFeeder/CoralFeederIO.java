package frc.robot.subsystems.CoralFeeder;

import org.littletonrobotics.junction.AutoLog;

public interface CoralFeederIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void stopIntake() {}

  default void setPID(double p, double i, double d) {}
}
