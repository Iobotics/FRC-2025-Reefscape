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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmConstants.*;

public class ArmIOSim implements ArmIO {
  private static final double autoStartAngle = Units.degreesToRadians(80.0);
  private double appliedVolts = 0.0;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNeoVortex(1),
          ArmConstants.GearRatio,
          0.1404,
          Units.inchesToMeters(5.927),
          Units.degreesToRadians(-180),
          Units.degreesToRadians(110.2),
          true,
          Units.degreesToRadians(110.2));

  private final PIDController controller;
  private double positionOffset = Units.degreesToRadians(110.2);

  private boolean controllerNeedsReset = false;
  private boolean closedLoop = true;
  private boolean wasNotAuto = true;

  public ArmIOSim() {
    controller = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // sim.update(Constants.loopPeriodSecs);
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // Reset at start of auto
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(autoStartAngle, 0.0);
      wasNotAuto = false;
    }
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    inputs.positionRad = positionOffset - sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();

    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    sim.update(Constants.loopPeriodSecs);
    sim.setInputVoltage(0);
  }

  @Override
  public void runSetpoint(double setpointDegrees, double feedfoward) {
    double setpoint = positionOffset - Units.degreesToRadians(setpointDegrees);

    double output = controller.calculate(sim.getAngleRads(), setpoint);
    appliedVolts = output - feedfoward;
    setVoltage(appliedVolts);
  }

  @Override
  public void setPID(double p, double i, double d) {
    controller.setPID(p, i, d);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }
}
