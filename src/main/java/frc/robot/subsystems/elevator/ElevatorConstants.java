package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorConstants {
  public static final double reduction = 3; // change to actual reduction
  public static final double rotationsToMeters =
      Math.PI * 2.0 * (1.0); // change 1.0 to the radius of the hub

  public static final double maxHeight = Units.inchesToMeters(72);

  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2.0, 2.0);

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case SIM -> new Gains(20.0, 0.0, 5.0, 0.0, 0.115, 0.0, 0.0);
        default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
