package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class RobotState {
  // 0 degrees is facing red alliance
  public enum reefZone {
    AB(Angle.ofBaseUnits(180, Units.Degrees)),
    CD(Angle.ofBaseUnits(120, Units.Degrees)),
    EF(Angle.ofBaseUnits(60, Units.Degrees)),
    GH(Angle.ofBaseUnits(0, Units.Degrees)),
    IJ(Angle.ofBaseUnits(300, Units.Degrees)),
    KL(Angle.ofBaseUnits(240, Units.Degrees));

    private Angle angle;

    reefZone(Angle angle) {
      this.angle = angle;
    }

    private Angle getAngle() {
      return angle;
    }
  }

  private static RobotState instance = null;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private RobotState() {}

  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  private void closestReefPose() {}

  // public Pose2d getReefGoalPose() {
  //     List<Pose2d> reefGoals = new ArrayList<Pose2d>();

  //     FieldConstants.reef
  //         .plus(
  //             new Transform2d(2,0.2, new Rotation2d(zone.getAngle()))
  //         );

  // }
}
