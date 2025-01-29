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

package frc.robot.subsystems.Algae;

import static frc.robot.subsystems.Algae.AlgaeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeIOSim implements AlgaeIO {
  private static final double autoStartAngle = Units.degreesToRadians(80.0);
  private double appliedVolts = 0.0;

  private final DCMotorSim sim =
   new DCMotorSim(
  DCMotorSim.getNEO(1),
GearRatio,
0.00912890116,
Armlength,
minAngle.getRadians(),
maxAngle.getRadians(),
true,
Units.degreesToRadians(0.0));
   

 private final PIDController controller;
  private double appliedVoltage = 0.0;
  private double positionOffset = 0.0;

  private boolean controllerNeedsReset = false;
  private boolean closedLoop = true;
  private boolean wasNotAuto = true;

  public AlgaeIOSim() {
    controller = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
    setPosition(0.0);
      }
    
      private void setPosition(double d) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
      }
    
    
      @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    sim.setInput(appliedVolts);
    sim.update(0.02);
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

    inputs.positionRad = sim.getAngularPositionRad() + positionOffset;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();

    inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.torqueCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.tempCelcius = new double[] {0.0};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
@Override
public void runSetpoint(double setpointRads, double feedforward){
  if (!closedLoop) {
    controllerNeedsReset = true;
    closedLoop = true;
  }
  if (controllerNeedsReset) {
    controller.reset();
    controllerNeedsReset = false;
  }
  runVolts(controller.calculate(sim.getAngleRads(), setpointRads + positionOffset) + feedforward);
  }
}
