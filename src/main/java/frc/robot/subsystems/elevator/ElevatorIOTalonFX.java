package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.reduction;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Hardware
  private final TalonFX main;
  private final TalonFX follower;

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
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private final PositionVoltage positionControl = new PositionVoltage(0.0);

  public ElevatorIOTalonFX() {
    main = new TalonFX(19);
    follower = new TalonFX(20);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = 40; // change later
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    main.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    follower.setControl(new Follower(19, false));

    positionRotations = main.getPosition();
    velocityRps = main.getVelocity();
    appliedVoltage = main.getMotorVoltage();
    supplyCurrent = main.getSupplyCurrent();
    torqueCurrent = main.getTorqueCurrent();
    tempCelsius = main.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);

    main.optimizeBusUtilization(0, 1.0);
  }

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
    inputs.positionRotations = (positionRotations.getValueAsDouble()) / reduction;
    inputs.positionMeters =
        (positionRotations.getValueAsDouble() / reduction) * ElevatorConstants.rotationsToMeters;
    inputs.velocityMeters =
        (velocityRps.getValueAsDouble() / reduction) * ElevatorConstants.rotationsToMeters;
    // inputs.velocityRps =
    //     velocityRps.getValueAsDouble() / reduction;
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    // inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void stop() {
    main.setControl(neutralOut);
  }

  @Override
  public void runVolts(double volts) {
    // volts = MathUtil.clamp(volts, -1.2, 1.2);
    main.setVoltage(volts);
  }

  @Override
  public void setPID(double p, double i, double d) {
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;

    main.getConfigurator().apply(config);
    // controller = new PIDController(p, i, d);
  }

  @Override
  public void runSetpoint(double setpointMeters, double feedforward) {
    double setpointRotations = (setpointMeters / ElevatorConstants.rotationsToMeters) * reduction;

    main.setControl(positionControl.withPosition(setpointRotations).withFeedForward(feedforward));
  }

  // @Override
  // public void (boolean enable) {
  //   talon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.talon }
}
