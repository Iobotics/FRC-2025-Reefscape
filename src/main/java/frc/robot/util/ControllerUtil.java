package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class ControllerUtil {
  public static Rotation2d snapToReef(double x, double y) {
    double angle = Math.atan2(y, x) - (Math.PI / 2);
    Logger.recordOutput("preSnap", angle);
    angle = Math.round(angle / (Math.PI / 3)) * (Math.PI / 3);
    angle +=
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
            ? Math.PI
            : 0;
    // angle += DriverStation.getAlliance() ? Math.PI;
    Logger.recordOutput("postSnap", angle);

    return new Rotation2d(angle);
  }
}
