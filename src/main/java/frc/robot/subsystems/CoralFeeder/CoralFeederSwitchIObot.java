package frc.robot.subsystems.CoralFeeder;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import java.time.Duration;

public class CoralFeederSwitchIObot implements CoralFeederSwitchIO {
  private final DigitalInput intakeSwitch = new DigitalInput(CoralFeeder_Constants.switchChannel);
  private final DigitalGlitchFilter glitchFilter = new DigitalGlitchFilter();

  public CoralFeederSwitchIObot() {
    glitchFilter.setPeriodNanoSeconds(Duration.ofMillis(1).toNanos());
    glitchFilter.add(intakeSwitch);
  }

  @Override
  public void updateInputs(CoralFeederSwitchIOInputs inputs) {
    inputs.intakeSwitchState = !intakeSwitch.get();
  }
}
