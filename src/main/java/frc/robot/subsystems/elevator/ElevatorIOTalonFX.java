package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.motionMagicConstraints;
import static frc.robot.subsystems.elevator.ElevatorConstants.reduction;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.elevator.ElevatorConstants.motionMagicConstraints;
import frc.robot.util.EqualsUtil;
import frc.robot.util.PhoenixUtil;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Hardware
  private final TalonFX motor1;
  private final TalonFX motor2;
  private final TalonFX motor3;
  private final TalonFX motor4;

  // Status Signals
  private final List<StatusSignal<Angle>> positionRotations;
  private final List<StatusSignal<AngularVelocity>> velocityRps;
  private final List<StatusSignal<Voltage>> appliedVoltage;
  private final List<StatusSignal<Current>> supplyCurrent;
  private final List<StatusSignal<Current>> torqueCurrent;
  private final List<StatusSignal<Temperature>> tempCelsius;
  private final List<StatusSignal<Double>> setpointRotations;

  private double goalPositionRotations;

  // Control
  private final NeutralOut neutralOut = new NeutralOut();
  private final PositionVoltage positionControl = new PositionVoltage(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0);
  private final Follower followerControl = new Follower(19, false);
  private final PositionTorqueCurrentFOC positionCurrentControl = new PositionTorqueCurrentFOC(0.0);

  private final DynamicMotionMagicTorqueCurrentFOC motionMagicControl =
      new DynamicMotionMagicTorqueCurrentFOC(
          0.0,
          motionMagicConstraints.velocity(),
          motionMagicConstraints.acceleration(),
          motionMagicConstraints.jerk());

  private TalonFXConfiguration config = new TalonFXConfiguration();

  public ElevatorIOTalonFX() {
    motor1 = new TalonFX(19, "Carnivore");
    motor2 = new TalonFX(20, "Carnivore");
    motor3 = new TalonFX(21, "Carnivore");
    motor4 = new TalonFX(22, "Carnivore");

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // config.CurrentLimits.SupplyCurrentLimit = 100; // change later
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.CurrentLimits.SupplyCurrentLowerLimit = 60;
    // config.CurrentLimits.SupplyCurrentLowerTime = 0.0;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -20;
    // config.CurrentLimits.StatorCurrentLimit = 160;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // motion magic configs
    config.MotionMagic.MotionMagicAcceleration = 10;
    config.MotionMagic.MotionMagicCruiseVelocity = 20;
    config.MotionMagic.MotionMagicJerk = 1000;

    motor1.getClosedLoopReference();

    PhoenixUtil.tryUntilOk(5, () -> motor1.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> motor2.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> motor3.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> motor4.getConfigurator().apply(config));

    motor2.setControl(followerControl);
    motor3.setControl(followerControl);
    motor4.setControl(followerControl);

    positionRotations = List.of(motor1.getPosition(), motor2.getPosition(),motor3.getPosition(),motor4.getPosition());
    velocityRps = List.of(motor1.getVelocity(), motor2.getVelocity(), motor3.getVelocity(), motor4.getVelocity());
    appliedVoltage = List.of(motor1.getMotorVoltage(), motor2.getMotorVoltage(), motor3.getMotorVoltage(), motor4.getMotorVoltage());
    supplyCurrent = List.of(motor1.getSupplyCurrent(), motor2.getSupplyCurrent(), motor3.getSupplyCurrent(), motor4.getSupplyCurrent());
    torqueCurrent = List.of(motor1.getTorqueCurrent(), motor2.getTorqueCurrent(), motor3.getTorqueCurrent(), motor4.getTorqueCurrent());
    tempCelsius = List.of(motor1.getDeviceTemp(), motor2.getDeviceTemp(), motor3.getDeviceTemp(), motor4.getDeviceTemp());
    setpointRotations = List.of(motor1.getClosedLoopReference(), motor2.getClosedLoopReference(), motor3.getClosedLoopReference(), motor4.getClosedLoopReference());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        positionRotations.get(0),
        positionRotations.get(1),
        positionRotations.get(2),
        positionRotations.get(3),

        velocityRps.get(0),
        velocityRps.get(1),
        velocityRps.get(2),
        velocityRps.get(3),

        appliedVoltage.get(0),
        appliedVoltage.get(1),
        appliedVoltage.get(2),
        appliedVoltage.get(3),

        supplyCurrent.get(0),
        supplyCurrent.get(1),
        supplyCurrent.get(2),
        supplyCurrent.get(3),

        torqueCurrent.get(0),
        torqueCurrent.get(1),
        torqueCurrent.get(2),
        torqueCurrent.get(3),

        tempCelsius.get(0),
        tempCelsius.get(1),
        tempCelsius.get(2),
        tempCelsius.get(3));

    motor1.optimizeBusUtilization(0, 1.0);
    motor2.optimizeBusUtilization(0, 1.0);
    motor3.optimizeBusUtilization(0, 1.0);
    motor4.optimizeBusUtilization(0, 1.0);

    goalPositionRotations = 0.0;
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motor1Connected =
        BaseStatusSignal.refreshAll(
                positionRotations.get(0),
                velocityRps.get(0),
                appliedVoltage.get(0),
                supplyCurrent.get(0),
                torqueCurrent.get(0),
                tempCelsius.get(0),
                setpointRotations.get(0))
            .isOK();

    inputs.motor2Connected =
        BaseStatusSignal.refreshAll(
                appliedVoltage.get(1),
                velocityRps.get(1),
                supplyCurrent.get(1),
                torqueCurrent.get(1),
                tempCelsius.get(1),
                setpointRotations.get(1))
            .isOK();

    inputs.motor3Connected =
    BaseStatusSignal.refreshAll(
            appliedVoltage.get(2),
            velocityRps.get(2),
            supplyCurrent.get(2),
            torqueCurrent.get(2),
            tempCelsius.get(2),
            setpointRotations.get(2))
        .isOK();
    inputs.motor4Connected =
        BaseStatusSignal.refreshAll(
                appliedVoltage.get(3),
                velocityRps.get(3),
                supplyCurrent.get(3),
                torqueCurrent.get(3),
                tempCelsius.get(3),
                setpointRotations.get(3))
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
    inputs.setpointRotations =
        setpointRotations.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  @Override
  public void stop() {
    motor1.setControl(neutralOut);
  }

  @Override
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        motor1.getPosition().getValueAsDouble(), goalPositionRotations, 1.0);
  }

  @Override
  public void runVolts(double volts) {
    motor1.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCurrent(double amps) {
    motor1.setControl(currentControl.withOutput(amps).withMaxAbsDutyCycle(0.2));
  }

  @Override
  public void setPID(double p, double i, double d, double v, double s, double a, double g) {
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    config.Slot0.kS = s;
    config.Slot0.kV = v;
    config.Slot0.kA = a;
    config.Slot0.kG = g;

    PhoenixUtil.tryUntilOk(5, () -> motor1.getConfigurator().apply(config));
  }

  @Override
  public void setMotionMagicConstraints(double velocity, double acceleration, double jerk) {
    config.MotionMagic.MotionMagicAcceleration = acceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = velocity;
    config.MotionMagic.MotionMagicJerk = jerk;

    PhoenixUtil.tryUntilOk(5, () -> motor1.getConfigurator().apply(config));

    motionMagicControl.Velocity = velocity;
    motionMagicControl.Acceleration = acceleration;
    motionMagicControl.Jerk = jerk;
  }

  @Override
  public void runSetpoint(double setpointMeters, double feedforward) {
    double setpoint = (setpointMeters / ElevatorConstants.rotationsToMeters) * reduction;

    Logger.recordOutput("Elevator/SetpointRotations", setpoint);
    // motor1.setControl()

    motor1.setControl( // kG 0.5 kV 1000.
        positionControl.withPosition(setpoint).withEnableFOC(true).withFeedForward(feedforward));

    //
    // motor1.setControl(
    //     positionCurrentControl
    //         .withPosition(Angle.ofBaseUnits(setpointRotations, Units.Rotations))
    //         .withFeedForward(feedforward));
  }

  @Override
  public void runSetpointMotionMagic(double setpointMeters, double feedforward) {
    goalPositionRotations = (setpointMeters / ElevatorConstants.rotationsToMeters) * reduction;
    Logger.recordOutput("Elevator/SetpointRotations", goalPositionRotations);
    motor1.setControl(
        motionMagicControl
            .withPosition((setpointMeters / ElevatorConstants.rotationsToMeters) * reduction)
            .withFeedForward(feedforward)
            .withSlot(0));
    if (goalPositionRotations == 0 && positionRotations.get(0).getValueAsDouble() < 0.4) {
      motor1.setControl(neutralOut);
    }
  }

  // @Override
  // public void (boolean enable) {
  //   talon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.talon }
}
