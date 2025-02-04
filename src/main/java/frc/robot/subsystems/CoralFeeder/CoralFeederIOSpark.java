package frc.robot.subsystems.CoralFeeder;

import static frc.robot.subsystems.CoralFeeder.CoralFeeder_Constants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.function.DoubleSupplier;

/**
 * This roller implementation is for Spark devices. It defaults to brushless control, but can be
 * easily adapted for a brushed motor. A Spark Flex can be used by swapping all instances of
 * "SparkMax" with "SparkFlex".
 */
public class CoralFeederIOSpark implements CoralFeederIO {
  private final SparkFlex Intake =
      new SparkFlex(CoralFeeder_Constants.intakeId, MotorType.kBrushless);
  private final RelativeEncoder encoder = Intake.getEncoder();

  public CoralFeederIOSpark() {
    var config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / motorReduction) // Rotor Rotations -> Roller Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    tryUntilOk(
        Intake,
        5,
        () ->
            Intake.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    ifOk(Intake, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(Intake, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        Intake,
        new DoubleSupplier[] {Intake::getAppliedOutput, Intake::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(Intake, Intake::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    Intake.setVoltage(volts);
  }
}
