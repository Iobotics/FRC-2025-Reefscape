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

import static frc.robot.subsystems.Arm.ArmConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;

/**
 * This roller implementation is for Spark devices. It defaults to brushless control, but can be
 * easily adapted for a brushed motor. A Spark Flex can be used by swapping all instances of
 * "SparkMax" with "SparkFlex".
 */
public class ArmIOSparkFlex implements ArmIO {
  // convert to degrees using gear box
  // right arm motor declaration
  private final SparkFlex Arm = new SparkFlex(ArmCANID, MotorType.kBrushless);
  private final AbsoluteEncoder encoder = Arm.getAbsoluteEncoder();
  private final SparkClosedLoopController pid = Arm.getClosedLoopController();
  // private final ArmFeedforward ArmFeedfoward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0, 0.0);

  // PID constants
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxOutput = 1.0;
  private static final double kMinOutput = -1.0;

  private SparkFlexConfig config;

  public ArmIOSparkFlex() {

    // double targetPosition = 5000;
    // Arm.getEncoder().setPosition(0);
    // motor.set(ControlType)
    config = new SparkFlexConfig();
    config.inverted(true).idleMode(IdleMode.kBrake);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .pid(kP, kI, kD)
        .outputRange(-1, 1);
    config.smartCurrentLimit(80);
    tryUntilOk(
        Arm,
        5,
        () ->
            Arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setPID(double p, double i, double d) {
    config.closedLoop.pid(p, i, d);

    tryUntilOk(
        Arm,
        5,
        () ->
            Arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadPerSec = Units.rotationsToRadians(encoder.getVelocity());
    inputs.appliedVolts = Arm.getAppliedOutput() * 12;
    inputs.currentAmps = Arm.getOutputCurrent();
  }

  @Override
  public void runSetpoint(double setpointDegrees, double ffVolts) {
    double setpoint = Units.degreesToRotations(setpointDegrees);
    pid.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
  }

  @Override
  public void setVoltage(double voltage) {
    Arm.set(voltage / 12);
  }
  /*
  public void setPosition(double position) {
    encoder.setPosition(position);
  }

  */
  @Override
  public void stop() {
    Arm.stopMotor();
  }

  //   @Override
  //   public void runSetpoint(double setpointRads, double ff){
  //     double feedforward = ArmFeedfoward.calculate(setpointRads, 0);
  //     pid.setReference(
  //       setpointRads, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
  // }

}

  // DELMAR ROBOTICS ENGINEERS AT MADE 2024 ROBOT    LOOK AT GITHUB
  // NRG ROBOT 2025
