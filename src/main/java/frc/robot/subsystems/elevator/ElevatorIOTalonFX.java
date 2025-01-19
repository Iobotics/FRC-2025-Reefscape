package frc.robot.subsystems.elevator;

import frc.robot.Constants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Hardware
  private final TalonFX talon;

  // Status Signals
  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // Control
  // private final TorqueCurrentFOC currentControl = new
  // TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralOut = new NeutralOut();

  public ElevatorIOTalonFX() {
    talon = new TalonFX(Constants.elevatorID); // change later

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 40; // change later
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    positionRotations = talon.getPosition();
    velocityRps = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);

    talon.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                positionRotations,
                velocityRps,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)
            .isOK();
    // inputs.positionRads =
    //     Units.rotationsToRadians(positionRotations.getValueAsDouble()) / reduction;
    // inputs.velocityRps =
    //     velocityRps.getValueAsDouble() / reduction;
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    // inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  public void runVolts(double volts) {
    talon.setVoltage(volts);
  }

  // @Override
  // public void setBrakeMode(boolean enable) {
  //   talon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  // }
}
