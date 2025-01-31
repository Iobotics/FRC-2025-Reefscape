package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  ElevatorSim elevator =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              DCMotor.getKrakenX60(2), 7.2, Units.inchesToMeters(0.75), reduction),
          DCMotor.getKrakenX60(2),
          0.,
          ElevatorConstants.maxHeight,
          true,
          0.,
          new double[] {0.01, 0.01});

  private PIDController controller;
  private boolean controllerNeedsReset = false;
  private boolean closedLoop = true;

  private double appliedVoltage = 0.0;

  public ElevatorIOSim() {
    controller = new PIDController(gains.kP(), gains.kI(), gains.kD());
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // inputs.positionMeters = elevator.getPositionMeters();
    // inputs.velocityMeters = elevator.getVelocityMetersPerSecond();
    // inputs.supplyCurrentAmps = elevator.getCurrentDrawAmps();
    // inputs.torqueCurrentAmps = elevator.getCurrentDrawAmps();
    // inputs.appliedVolts = appliedVoltage;

    elevator.update(Constants.loopPeriodSecs);
    elevator.setInputVoltage(0.0);
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    elevator.setInputVoltage(appliedVoltage);
  }

  @Override
  public void runSetpoint(double setpointMeters) {
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }
    runVolts(controller.calculate(elevator.getPositionMeters(), setpointMeters));
  }

  // @Override
  // public void setPID(double p, double i, double d) {
  //   controller.setPID(p, i, d);
  // }
}
