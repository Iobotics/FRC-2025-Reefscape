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
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;

/**
 * This roller implementation is for Spark devices. It defaults to brushless control, but can be
 * easily adapted for a brushed motor. A Spark Flex can be used by swapping all instances of
 * "SparkMax" with "SparkFlex".
 */
public class AlgaeIOSpark implements AlgaeIO {
  private PIDController controller;

  //right arm motor declaration
  private final SparkMax Arm = new SparkMax(AlgaeCANID, MotorType.kBrushless);
  private final RelativeEncoder ArmEncoder = Arm.getEncoder();
  double currentPosition = Arm.getEncoder().getPosition();
 private SparkClosedLoopController rightClosedLoopController = Arm.getClosedLoopController();


  //PID constants
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxOutput = 1.0;
  private static final double kMinOutput = -1.0;

  public AlgaeIOSpark() {
//right arm configuration
double targetPosition = 5000;
Arm.getEncoder().setPosition(0);
//motor.set(ControlType)


SparkMaxConfig rightConfig = new SparkMaxConfig();
 rightConfig.inverted(true).idleMode(IdleMode.kCoast);
    rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);
    Arm.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    ifOk(Arm, ArmEncoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(Arm, ArmEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        Arm,
        new DoubleSupplier[] {
          Arm::getAppliedOutput, Arm::getBusVoltage
        },
        (values) -> inputs.appliedVolts = values[0] * values[1]);

    ifOk(
      Arm,
      Arm::getOutputCurrent,
      (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    Arm.setVoltage(volts);
  }
}
