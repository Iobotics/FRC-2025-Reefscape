package frc.robot.subsystems.climb.pivot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class PivotIOSparkFlex implements PivotIO {
  SparkFlex motor;
  SparkClosedLoopController controller;

  SparkFlexConfig config = new SparkFlexConfig();

  public PivotIOSparkFlex() {
    motor = new SparkFlex(30, MotorType.kBrushless);
    controller = motor.getClosedLoopController();
  }

  @Override
  public void setVoltage(double voltage) {
    controller.setReference(voltage, ControlType.kVoltage);
  }

  @Override
  public void stop() {}

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRadians = motor.getEncoder().getPosition();
    inputs.appliedVoltage = motor.getAppliedOutput() * 12;
    inputs.current = motor.getOutputCurrent();
    inputs.temperature = motor.getMotorTemperature();
  }
}
