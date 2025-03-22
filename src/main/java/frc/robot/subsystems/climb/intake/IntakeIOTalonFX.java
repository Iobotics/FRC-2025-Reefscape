package frc.robot.subsystems.climb.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import java.util.List;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX leftMotor = new TalonFX(42, "Carnivore");
  private final TalonFX rightMotor = new TalonFX(41, "Carnivore");

  private TalonFXConfiguration config = new TalonFXConfiguration();

  private final List<StatusSignal<AngularVelocity>> velocityRps;
  private final List<StatusSignal<Voltage>> appliedVoltage;
  private final List<StatusSignal<Current>> supplyCurrent;
  private final List<StatusSignal<Current>> torqueCurrent;
  private final List<StatusSignal<Temperature>> tempCelsius;

  public IntakeIOTalonFX() {
    velocityRps = List.of(leftMotor.getVelocity(), rightMotor.getVelocity());
    appliedVoltage = List.of(leftMotor.getMotorVoltage(), rightMotor.getMotorVoltage());
    supplyCurrent = List.of(leftMotor.getSupplyCurrent(), rightMotor.getSupplyCurrent());
    torqueCurrent = List.of(leftMotor.getTorqueCurrent(), rightMotor.getTorqueCurrent());
    tempCelsius = List.of(leftMotor.getDeviceTemp(), rightMotor.getDeviceTemp());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        velocityRps.get(0),
        velocityRps.get(1),
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        torqueCurrent.get(0),
        torqueCurrent.get(1),
        tempCelsius.get(0),
        tempCelsius.get(1));

    leftMotor.optimizeBusUtilization(0, 1.0);
    rightMotor.optimizeBusUtilization(0, 1.0);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config));

    leftMotor.setControl(new Follower(41, true));
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void stop() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorConnected = leftMotor.isConnected() && rightMotor.isConnected();
  }
}
