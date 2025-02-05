package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorConstants {
  public static final double reduction = 5; // change to actual reduction
  public static final double drumDiameter = Units.inchesToMeters(1.5);
  public static final double rotationsToMeters = Math.PI * drumDiameter * 2;
  public static final double maxHeight = Units.inchesToMeters(72);

  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2.0, 2.0);

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case SIM -> new Gains(20.0, 0.0, 5.0, 0.0, 0.115, 0.0, 0.0);
        default -> new Gains(10.0, 3.0, 1.4, 0.0, 2.8, 0.6, 5.0);
      };

  public static final motionMagicConstraints motionMagicConstraints =
      new motionMagicConstraints(100.0, 100.0, 1000.0);

  public record motionMagicConstraints(double velocity, double acceleration, double jerk) {}

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
