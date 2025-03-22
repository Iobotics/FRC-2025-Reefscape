package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.intake.Intake;
import frc.robot.subsystems.climb.pivot.Pivot;

public class Climb extends SubsystemBase {
  private Intake intake;
  private Pivot pivot;

  public Climb(Intake intake, Pivot pivot) {
    this.intake = intake;
    this.pivot = pivot;
  }

  public void setIntakeVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  public void setPivotVoltage(double voltage) {
    pivot.setVoltage(voltage);
  }

  public void stopPivot() {
    pivot.setVoltage(0.0);
  }

  public void stopIntake() {
    intake.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
