package frc.robot.subsystems.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Sensor.Sensor;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralManipulator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CoralManipulatorIO io;

  private final CoralManipulatorIOInputsAutoLogged inputs =
      new CoralManipulatorIOInputsAutoLogged();

  public CoralManipulator(CoralManipulatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    io.updateInputs(inputs);
    Logger.processInputs("CoralManipulator", inputs);
  }

  @AutoLogOutput
  public void runOutake(double percentVolts) {
    io.setVoltage(percentVolts * 12);
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }

  public Command getCommand(Sensor coralSwitch) {
    return new Command() {
      @Override
      public void execute() {
        runOutake(0.5);
      }

      @Override
      public boolean isFinished() {
        return !coralSwitch.getSwitch();
      }

      @Override
      public void end(boolean interrupted) {
        runOutake(0);
      }
    };
  }
}
