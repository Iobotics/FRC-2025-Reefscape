package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private static final LoggedTunableNumber maxVelocityMotionMagic =
      new LoggedTunableNumber("Elevator/MotionMagic/Velocity", motionMagicConstraints.velocity());
  private static final LoggedTunableNumber maxAccelerationMotionMagic =
      new LoggedTunableNumber(
          "Elevator/MotionMagic/Acceleration", motionMagicConstraints.acceleration());
  private static final LoggedTunableNumber maxJerkMotionMagic =
      new LoggedTunableNumber("Elevator/MotionMagic/Jerk", motionMagicConstraints.jerk());

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
    SCOREL1(new LoggedTunableNumber("Elevator/ScoreL1", 0.1)),
    SCOREL2(new LoggedTunableNumber("Elevator/ScoreL2", 0.29)),
    SCOREL3(new LoggedTunableNumber("Elevator/ScoreL3", 0.79)),
    SCOREL4(new LoggedTunableNumber("Elevator/ScoreL4", 1.26)),
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

  boolean closedLoop = true;

  public void setGoal(Goal newGoal) {
    closedLoop = true;
    goal = newGoal;
  }

  public void returnToHome() {
    goal = Goal.STOW;
  }

  public void manualCurrent(double amps) {
    closedLoop = false;
    io.runCurrent(amps);
  }

  public void stop() {
    closedLoop = false;
    io.stop();
  }

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
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get(), kV.get(), kS.get(), kA.get(), kG.get()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA,
        kG);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            profile =
                new TrapezoidProfile(new Constraints(maxVelocity.get(), maxAcceleration.get())),
        maxVelocity,
        maxAcceleration);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.setMotionMagicConstraints(
                maxVelocityMotionMagic.get(),
                maxAccelerationMotionMagic.get(),
                maxJerkMotionMagic.get()),
        maxVelocityMotionMagic,
        maxAccelerationMotionMagic,
        maxJerkMotionMagic);

    goalMeters = goal.getMeters();

    setpointState =
        profile.calculate(
            Constants.loopPeriodSecs, setpointState, new TrapezoidProfile.State(goalMeters, 0.0));

    // setpoint code, comment out if using motion magic
    // if (closedLoop) {
    //   io.runSetpoint(
    //       setpointState.position,
    //       kG.get()
    //           + kV.get() * setpointState.velocity
    //           + kS.get() * Math.signum(setpointState.velocity));
    // }

    // motion magic setpoint code
    if (closedLoop) {
      io.runSetpointMotionMagic(goalMeters, 0);
    }

    Logger.recordOutput("Elevator/SetpointPos", setpointState.position);
    Logger.recordOutput("Elevator/GoalPos", goalMeters);
    // measuredVisualizer.update(inputs.positionMeters);
    setpointVisualizer.update(setpointState.position);
    goalVisualizer.update(goalMeters);
  }

  @AutoLogOutput
  public boolean atGoal(double tolerance) {
    return io.atGoal();
  }

  public boolean atGoal() {
    return atGoal(1e-6);
  }
}
