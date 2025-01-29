package frc.robot.subsystems.Algae;

import static frc.robot.subsystems.Algae.AlgaeConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
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
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber("Arm/LowerLimitDegrees", minAngle.getDegrees());
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber("Arm/UpperLimitDegrees", maxAngle.getDegrees());

  public enum Goal {
    DEFAULT(() -> 0),
    UNJAM_INTAKE(new LoggedTunableNumber("Arm/UnjamDegrees", 40.0)),
    STATION_INTAKE(new LoggedTunableNumber("Arm/StationIntakeDegrees", 45.0)),
    CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 20.0));

    private final DoubleSupplier armSetpointSupplier;

    private Goal(DoubleSupplier armSetpointSupplier) {
      this.armSetpointSupplier = armSetpointSupplier;
    }

    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }

  @AutoLogOutput private Goal goal = Goal.DEFAULT;

  public void setGoal(Goal newGoal) {
    goal = newGoal;
  }

  private boolean characterizing = false;

  private final AlgaeIO io;

  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  public Algae(AlgaeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  }

  public void runOutake(double percentVolts) {
    io.setVoltage(percentVolts * 12);
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }
}
