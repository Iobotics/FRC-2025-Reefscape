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

package frc.robot.subsystems.CoralManipulator;

import static frc.robot.subsystems.CoralManipulator.CoralManipulatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CoralManipulatorIOSim implements CoralManipulatorIO {

  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.004, motorReduction),
          DCMotor.getNeoVortex(1));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(CoralManipulatorIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setOutakeSpeed(double targetRPM) {
    setVoltage(targetRPM * motorReduction * 2.0 * Math.PI / 60.0);
  }
}
