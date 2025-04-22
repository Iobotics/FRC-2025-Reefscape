package frc.robot.subsystems.Funnel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class FunnelIOTalonFX implements FunnelIO {
  TalonFX motor;

  TalonFXConfiguration config = new TalonFXConfiguration();

  StatusSignal<Voltage> suppliedVoltage;
  StatusSignal<Current> supplyCurrent;
  StatusSignal<Current> torqueCurrent;
  StatusSignal<AngularVelocity> velocityRotsPerSec;

  VoltageOut voltageControl = new VoltageOut(0.0);

  public FunnelIOTalonFX() {
    motor = new TalonFX(50, "Carnivore");

    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    suppliedVoltage = motor.getSupplyVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    velocityRotsPerSec = motor.getVelocity();

    StatusSignal.setUpdateFrequencyForAll(
        50, suppliedVoltage, supplyCurrent, torqueCurrent, velocityRotsPerSec);

    motor.optimizeBusUtilization(0, 1.0);
  }

  public void updateInputs(FunnelIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                suppliedVoltage, supplyCurrent, torqueCurrent, velocityRotsPerSec)
            .isOK();

    inputs.appliedVolts = suppliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.velocityRotsPerSec = velocityRotsPerSec.getValueAsDouble();
  }

  @Override
  public void runVoltage(double volts) {
    motor.setControl(voltageControl.withOutput(volts));
  }
}
