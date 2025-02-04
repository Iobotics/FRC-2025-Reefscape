package frc.robot.subsystems.CoralFeeder;

import org.littletonrobotics.junction.AutoLog;

public interface CoralFeederSwitchIO {
  @AutoLog
  class CoralFeederSwitchIOInputs {
    boolean intakeSwitchState = false;
  }

  default void updateInputs(CoralFeederSwitchIOInputs inputs) {}
}
