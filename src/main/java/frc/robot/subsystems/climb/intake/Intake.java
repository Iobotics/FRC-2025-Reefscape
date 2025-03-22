package frc.robot.subsystems.climb.intake;

public class Intake {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
  }
}
