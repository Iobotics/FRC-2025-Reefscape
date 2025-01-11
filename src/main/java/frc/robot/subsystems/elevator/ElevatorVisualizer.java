package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class ElevatorVisualizer {
  private LoggedMechanism2d mechanism;
  private LoggedMechanismLigament2d elevator;
  private String key;

  public ElevatorVisualizer(String key, Color color) {

    mechanism = new LoggedMechanism2d(2.0, 2.0, new Color8Bit(Color.kWhite));
    elevator = new LoggedMechanismLigament2d("elevator", 1.0, 90, 8, new Color8Bit(color));
    mechanism.getRoot("pivot", 0.5, 0).append(elevator);
    this.key = key;
  }

  public void update(double length) {
    elevator.setLength(0.1 + length);
    Logger.recordOutput("Elevator/Mechanism2d" + key, mechanism);
  }
}
