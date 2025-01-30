package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.reduction;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Hardware
  private final TalonFX main;
  private final TalonFX follower;

  // Status Signals
  private final List<StatusSignal<Angle>> positionRotations;
  private final List<StatusSignal<AngularVelocity>> velocityRps;
  private final List<StatusSignal<Voltage>> appliedVoltage;
  private final List<StatusSignal<Current>> supplyCurrent;
  private final List<StatusSignal<Current>> torqueCurrent;
  private final List<StatusSignal<Temperature>> tempCelsius;

  // Control
  private final NeutralOut neutralOut = new NeutralOut();
  private final PositionVoltage positionControl = new PositionVoltage(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC positionCurrentControl = new PositionTorqueCurrentFOC(0.0);

  public ElevatorIOTalonFX() {
    main = new TalonFX(19, "Carnivore");
    follower = new TalonFX(20, "Carnivore");

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = 80; // change later
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80;
    main.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    follower.setControl(new Follower(19, false));

    positionRotations = List.of(main.getPosition(), follower.getPosition());
    velocityRps = List.of(main.getVelocity(), follower.getVelocity());
    appliedVoltage = List.of(main.getMotorVoltage(), follower.getMotorVoltage());
    supplyCurrent = List.of(main.getSupplyCurrent(), follower.getSupplyCurrent());
    torqueCurrent = List.of(main.getTorqueCurrent(), follower.getTorqueCurrent());
    tempCelsius = List.of(main.getDeviceTemp(), follower.getDeviceTemp());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        positionRotations.get(0),
        positionRotations.get(1),
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

    main.optimizeBusUtilization(0, 1.0);
    follower.optimizeBusUtilization(0, 1.0);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(
                positionRotations.get(0),
                velocityRps.get(0),
                appliedVoltage.get(0),
                supplyCurrent.get(0),
                torqueCurrent.get(0),
                tempCelsius.get(0))
            .isOK();

    inputs.followerMotorConnected =
        BaseStatusSignal.refreshAll(
                appliedVoltage.get(1),
                velocityRps.get(1),
                supplyCurrent.get(1),
                torqueCurrent.get(1),
                tempCelsius.get(1))
            .isOK();

    inputs.positionRotations =
        positionRotations.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.velocityMeters =
        velocityRps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.appliedVolts =
        appliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.supplyCurrentAmps =
        supplyCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.torqueCurrentAmps =
        torqueCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.positionMeters =
        new double[] {
          (inputs.positionRotations[0] / reduction) * ElevatorConstants.rotationsToMeters,
          (inputs.positionRotations[0] / reduction) * ElevatorConstants.rotationsToMeters
        };
  }

  @Override
  public void stop() {
    main.setControl(neutralOut);
  }

  @Override
  public void runVolts(double volts) {
    main.setVoltage(volts);
  }

  @Override
  public void runCurrent(double amps) {
    main.setControl(currentControl.withOutput(amps).withMaxAbsDutyCycle(0.2));
  }

  @Override
  public void setPID(double p, double i, double d) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;

    main.getConfigurator().apply(config);
  }

  @Override
  public void runSetpoint(double setpointMeters, double feedforward) {
    double setpointRotations = (setpointMeters / ElevatorConstants.rotationsToMeters) * reduction;
    // main.setControl()

    // main.setControl(
    //     positionControl
    //         .withPosition(Angle.ofBaseUnits(setpointRotations, Units.Rotations))
    //         .withFeedForward(feedforward)
    //         .withEnableFOC(true));

    main.setControl(
        positionCurrentControl
            .withPosition(Angle.ofBaseUnits(setpointRotations, Units.Rotations))
            .withFeedForward(feedforward));
  }

  // @Override
  // public void (boolean enable) {
  //   talon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.talon }
}
