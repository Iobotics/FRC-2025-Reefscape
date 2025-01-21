package frc.robot.subsystems.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorIO.CoralManipulatorIOInputs;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CoralManipulator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CoralManipulatorIO io;

  private final CoralManipulatorIOInputs inputs = new CoralManipulatorIOInputs();

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxOutput = 1.0;
  private static final double kMinOutput = -1.0;

  public CoralManipulator(CoralManipulatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    io.updateInputs(inputs);
    Logger.processInputs("CoralManipulator", (LoggableInputs) inputs);
  }

  public Command runOutake(double speed) {
    return runEnd(() -> io.setOutakeSpeed(speed), () -> io.setOutakeSpeed(0.0));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }
}
