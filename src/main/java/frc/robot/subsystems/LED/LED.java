package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 178;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public final LEDPattern red;
  public final LEDPattern blue;
  public final LEDPattern black;
  public final LEDPattern green;
  public final LEDPattern yellow;

  public LED() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    // Create an LED pattern that sets the entire strip to solid red
    red = LEDPattern.solid(Color.kRed);
    blue = LEDPattern.solid(Color.kBlue);
    black = LEDPattern.solid(Color.kBlack);
    green = LEDPattern.solid(Color.kGreen);
    yellow = LEDPattern.solid(Color.kYellow);

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    // setDefaultCommand(onBlue());
    red.applyTo(m_buffer);
    m_led.setData(m_buffer);
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    m_led.setData(m_buffer);
  }

  public void applyLED(LEDPattern pattern) {
    pattern.applyTo(m_buffer);
    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> applyLED(pattern));
  }

  public Command onRed() {
    return run(() -> applyLED(this.red));
  }

  public Command onBlue() {
    return run(() -> applyLED(this.blue));
  }

  public Command onGreen() {
    return run(() -> applyLED(this.green));
  }
}
