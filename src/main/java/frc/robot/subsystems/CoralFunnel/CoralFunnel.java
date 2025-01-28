package frc.robot.subsystems.CoralFunnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class CoralFunnel extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CoralFunnelIO io;

  private final CoralFunnelIOInputsAutoLogged inputs = new CoralFunnelIOInputsAutoLogged();

  public CoralFunnel(CoralFunnelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    io.updateInputs(inputs);
    Logger.processInputs("CoralFunnel", inputs);
  }

  public void runFunnel(double percentVolts) {
    io.setVoltage(percentVolts * 12);
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }
}
