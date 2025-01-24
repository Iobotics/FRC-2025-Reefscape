package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Elevator/maxVelocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Elevator/maxAcceleration", profileConstraints.maxAcceleration);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", gains.kD());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Elevator/kG", gains.ffkG());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Elevator/kA", gains.ffkA());

  public final ElevatorIO io;
  public final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  ElevatorFeedforward feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private final ElevatorVisualizer measuredVisualizer;
  private final ElevatorVisualizer setpointVisualizer;
  private final ElevatorVisualizer goalVisualizer;

  private double goalMeters;

  public enum Goal {
    STOW(new LoggedTunableNumber("Elevator/Stow", 0.0)),
    SCOREL1(new LoggedTunableNumber("Elevator/ScoreL1", 0.5)),
    SCOREL2(new LoggedTunableNumber("Elevator/ScoreL2", 0.8)),
    SCOREL3(new LoggedTunableNumber("Elevator/ScoreL3", 1.1)),
    SCOREL4(new LoggedTunableNumber("Elevator/ScoreL4", 2.0)),
    INTAKE(new LoggedTunableNumber("Elevator/Intake", 0.2)),
    ;

    private final DoubleSupplier elevatorSetpointSupplier;

    private Goal(DoubleSupplier elevatorSetpointSupplier) {
      this.elevatorSetpointSupplier = elevatorSetpointSupplier;
    }

    private double getMeters() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput private Goal goal = Goal.STOW;

  public void setGoal(Goal newGoal) {
    goal = newGoal;
  }

  public void stop() {
    goal = Goal.STOW;
  }

  boolean closedLoop = true;

  public Elevator(ElevatorIO io) {
    this.io = io;
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    measuredVisualizer = new ElevatorVisualizer("Elevator/Measured", Color.kBlack);
    setpointVisualizer = new ElevatorVisualizer("Elevator/Setpoint", Color.kGreen);
    goalVisualizer = new ElevatorVisualizer("Elevator/Goal", Color.kRed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            profile =
                new TrapezoidProfile(new Constraints(maxVelocity.get(), maxAcceleration.get())),
        maxVelocity,
        maxAcceleration);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> feedforward = new ElevatorFeedforward(kG.get(), kS.get(), kV.get(), kA.get()),
        kG,
        kS,
        kV,
        kA);
    // LoggedTunableNumber.ifChanged(
    //     hashCode(), () ->
    // setGoal(setPointPositionMeters.get(),setPointVelocityMetersPerSecond.get()),
    // setPointPositionMeters,setPointVelocityMetersPerSecond);

    goalMeters = goal.getMeters();

    setpointState =
        profile.calculate(
            Constants.loopPeriodSecs, setpointState, new TrapezoidProfile.State(goalMeters, 0.0));

    io.runSetpoint(
        setpointState.position,
        feedforward.calculate(setpointState.position, setpointState.velocity));

    Logger.recordOutput("Elevator/SetpointPos", setpointState.position);
    Logger.recordOutput("Elevator/GoalPos", goalMeters);
    measuredVisualizer.update(inputs.positionMeters);
    setpointVisualizer.update(setpointState.position);
    goalVisualizer.update(goalMeters);
  }

  @AutoLogOutput
  public boolean atGoal(double tolerance) {
    return EqualsUtil.epsilonEquals(setpointState.position, goalMeters, tolerance);
  }

  public boolean atGoal() {
    return atGoal(1e-6);
  }
}
