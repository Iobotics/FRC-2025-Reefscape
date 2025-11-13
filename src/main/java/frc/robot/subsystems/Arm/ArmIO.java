// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean armConnect = true;
    public boolean absoluteEncoderConnected = true;

    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;

    public double absEncoderPositionRads = 0.0;
    public double relEncoderPositionRads = 0.0;

    // PID variables

  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void runSetpoint(double setpointDegrees, double feedfoward) {}

  /** Run motors at current */
  public default void runCurrent(double amps) {}

  /** Set brake mode enabled */
  public default void setBrakeMode(boolean enabled) {}

  public default void resetClosedLoop() {}

  /** Set PID values */
  public default void setPID(double p, double i, double d) {}

  /** Stops motors */
  public default void stop() {}
}
