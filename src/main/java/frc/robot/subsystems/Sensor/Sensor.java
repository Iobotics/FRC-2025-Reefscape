package frc.robot.subsystems.Sensor;

import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.AutoLogOutput;

public class Sensor {
  // Sensor
  public final DigitalInput coralSwitch = new DigitalInput(0);

  @AutoLogOutput(key = "Sensor/value")
  public boolean getSwitch() {
    return coralSwitch.get();
  }
}
