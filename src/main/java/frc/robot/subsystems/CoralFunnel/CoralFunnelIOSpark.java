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

package frc.robot.subsystems.CoralFunnel;

import static frc.robot.subsystems.CoralFunnel.CoralFunnelConstants.*;
import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

/**
 * This roller implementation is for Spark devices. It defaults to brushless control, but can be
 * easily adapted for a brushed motor. A Spark Flex can be used by swapping all instances of
 * "SparkMax" with "SparkFlex".
 */
public class CoralFunnelIOSpark implements CoralFunnelIO {
  private final SparkMax coralFunnel = new SparkMax(coralFunnelCanId, MotorType.kBrushless);
  private SparkClosedLoopController funnelClosedLoopController =
      coralFunnel.getClosedLoopController();
  private final RelativeEncoder funnelEncoder = coralFunnel.getEncoder();

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxOutput = 1.0;
  private static final double kMinOutput = -1.0;

  public CoralFunnelIOSpark() {
    // config
    SparkMaxConfig funnelConfig = new SparkMaxConfig();

    funnelConfig.inverted(true).idleMode(IdleMode.kCoast);
    funnelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    coralFunnel.configure(
        funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(CoralFunnelIOInputs inputs) {
    ifOk(coralFunnel, funnelEncoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(coralFunnel, funnelEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        coralFunnel,
        new DoubleSupplier[] {coralFunnel::getAppliedOutput, coralFunnel::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(coralFunnel, coralFunnel::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    coralFunnel.setVoltage(volts);
  }
}
