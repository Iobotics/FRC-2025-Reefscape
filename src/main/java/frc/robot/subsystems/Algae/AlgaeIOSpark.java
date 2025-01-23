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

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
/**
 * This roller implementation is for Spark devices. It defaults to brushless control, but can be
 * easily adapted for a brushed motor. A Spark Flex can be used by swapping all instances of
 * "SparkMax" with "SparkFlex".
 */
public class AlgaeIOSpark implements AlgaeIO {
  private static final double GEAR_RATIO = 75;

  // right arm motor declaration
  private final SparkFlex Arm = new SparkFlex(AlgaeCANID, MotorType.kBrushless);
  private final RelativeEncoder ArmEncoder = Arm.getEncoder();
  private final SparkClosedLoopController pid = Arm.getClosedLoopController();

  // PID constants
 
 
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxOutput = 1.0;
  private static final double kMinOutput = -1.0;

  public AlgaeIOSpark() {

    // double targetPosition = 5000;
    // Arm.getEncoder().setPosition(0);
    // motor.set(ControlType)
    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(false).idleMode(IdleMode.kCoast);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);
    Arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    ifOk(Arm, ArmEncoder::getPosition, (value) -> inputs.positionRad = Units.rotationsToRadians(ArmEncoder.getPosition() / GEAR_RATIO));
    ifOk(Arm, ArmEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        Arm,
        new DoubleSupplier[] {Arm::getAppliedOutput, Arm::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);

    ifOk(Arm, Arm::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

 /*  @Override
  public void setVoltage(double volts) {
    Arm.setVoltage(volts); */

 @Override
    public void runSetpoint(double setpointRads, double ffVolts) {
        pid.setReference(
            Units.radiansToRotations(setpointRads) * GEAR_RATIO,
            ControlType.kPosition,
            0,
            ffVolts,
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void runVolts(double volts) {
        Arm.setVoltage(volts);
    }

    @Override
    public void setPosition(double position){
ArmEncoder.setPosition(position);

    }

    @Override
    public void stop() {
        Arm.stopMotor();
    }

  }

  //DELMAR ROBOTICS ENGINEERS AT MADE 2024 ROBOT    LOOK AT GITHUB 