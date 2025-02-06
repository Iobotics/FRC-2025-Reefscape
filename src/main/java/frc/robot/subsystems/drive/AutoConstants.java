package frc.robot.subsystems.drive;

import frc.robot.Constants;

public class AutoConstants {
  public static final Gains translationGains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(10.0, 0.0, 0.0);
        default -> new Gains(0.5, 0.0, 0.0);
      };
  public static final Gains rotationGains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(8.0, 0.0, 0.0);
        default -> new Gains(0.0, 0.0, 0.0);
      };

  public record Gains(double kP, double kI, double kD) {}
}
