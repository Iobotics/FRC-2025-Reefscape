package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.reduction;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import java.util.List;
import org.littletonrobotics.junction.Logger;

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
  private final VoltageOut voltageControl = new VoltageOut(0.0);
  private final Follower followerControl = new Follower(19, false);
  private final PositionTorqueCurrentFOC positionCurrentControl = new PositionTorqueCurrentFOC(0.0);
  private TalonFXConfiguration config = new TalonFXConfiguration();

  public ElevatorIOTalonFX() {
    main = new TalonFX(19, "Carnivore");
    follower = new TalonFX(20, "Carnivore");

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // config.CurrentLimits.SupplyCurrentLimit = 100; // change later
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.CurrentLimits.SupplyCurrentLowerLimit = 60;
    // config.CurrentLimits.SupplyCurrentLowerTime = 0.0;
    // config.TorqueCurrent.PeakForwardTorqueCurrent = 80;
    // config.TorqueCurrent.PeakReverseTorqueCurrent = -80;
    // config.CurrentLimits.StatorCurrentLimit = 160;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    PhoenixUtil.tryUntilOk(5, () -> main.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));

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
    main.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCurrent(double amps) {
    main.setControl(currentControl.withOutput(amps).withMaxAbsDutyCycle(0.2));
  }

  @Override
  public void setPID(double p, double i, double d, double v, double s, double a, double g) {
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    // config.Slot0.kS = s;
    // config.Slot0.kV = v;
    // config.Slot0.kA = a;
    // config.Slot0.kG = g;

    PhoenixUtil.tryUntilOk(5, () -> main.getConfigurator().apply(config));
  }

  @Override
  public void runSetpoint(double setpointMeters, double feedforward) {
    double setpointRotations = (setpointMeters / ElevatorConstants.rotationsToMeters) * reduction;

    Logger.recordOutput("Elevator/SetpointRotations", setpointRotations);
    // main.setControl()

    main.setControl( // kG 0.5 kV 1000.
        positionControl
            .withPosition(Angle.ofBaseUnits(setpointRotations, Units.Rotations))
            .withEnableFOC(true)
            .withFeedForward(feedforward));

    follower.setControl(new StrictFollower(19));

    //
    // main.setControl(
    //     positionCurrentControl
    //         .withPosition(Angle.ofBaseUnits(setpointRotations, Units.Rotations))
    //         .withFeedForward(feedforward));
  }

  // @Override
  // public void (boolean enable) {
  //   talon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.talon }
}
