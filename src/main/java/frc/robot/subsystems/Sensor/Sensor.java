package frc.robot.subsystems.Sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class Sensor {
  // Sensor
  public final DigitalInput coralSwitch = new DigitalInput(0);

  public boolean getSwitch() {
    return coralSwitch.get();
  }
}
