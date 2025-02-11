package frc.robot.subsystems.Arm;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.subsystems.Arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", gains.kP());

  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Arm/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Arm/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Arm/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Arm/Gains/kG", gains.ffkG());
  private static final LoggedTunableNumber Acceleration =
      new LoggedTunableNumber("Arm/Acceleration", profileConstraints.maxAcceleration);
  private static final LoggedTunableNumber Velocity =
      new LoggedTunableNumber("Arm/Velocity", profileConstraints.maxVelocity);

  public enum Goalposition {
    DEFAULT(() -> 0),
    SCOREL4(new LoggedTunableNumber("Arm/ScoreL4", 25.0)),
    INTAKEALGAE(new LoggedTunableNumber("Arm/IntakeAlgae", 45.0)),
    CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 20.0));

    private final DoubleSupplier armSetpointSupplier;

    private Goalposition(DoubleSupplier armSetpointSupplier) {
      this.armSetpointSupplier = armSetpointSupplier;
    }

    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }

  @AutoLogOutput private Goalposition goalposition = Goalposition.DEFAULT;

  public void setGoal(Goalposition newGoal) {
    goalposition = newGoal;
  }

  @AutoLogOutput private double currentCompensation = 0.0;
  private TrapezoidProfile profile = new TrapezoidProfile(profileConstraints);
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private double goalAngle;
  private ArmFeedforward ff;

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;
  private BooleanSupplier halfStowSupplier = () -> true;
  private boolean brakeModeEnabled = true;

  private boolean wasNotAuto = false;

  private boolean characterizing = false;

  private final ArmIO io;

  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    goalAngle = goalposition.getRads();

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            profile =
                new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(Velocity.get(), Acceleration.get())),
        Velocity,
        Acceleration);
    setpointState =
        profile.calculate(
            Constants.loopPeriodSecs, setpointState, new TrapezoidProfile.State(goalAngle, 0.0));

    double ffVolts =
        ff.calculate(
            (Math.PI / 2) + degreesToRadians(20.2) - setpointState.position,
            setpointState.velocity);

    Logger.recordOutput("Arm/FFVolts", ffVolts);
    Logger.recordOutput("Arm/GoalAngle", goalAngle);
    Logger.recordOutput("Arm/SetpointAngle", setpointState.position);

    setBrakeMode(!coastSupplier.getAsBoolean());
    io.runSetpoint(Units.radiansToDegrees(setpointState.position), ffVolts);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Arm/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(setpointState.position, goalAngle, 0.1);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public Command getSetpointCommand(Goalposition goal) {
    return new Command() {
      @Override
      public void initialize() {
        setGoal(goal);
      }

      @Override
      public boolean isFinished() {
        return atGoal();
      }
    };
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }
}
