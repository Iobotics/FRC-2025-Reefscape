package frc.robot;

import static edu.wpi.first.math.util.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  // 0 degrees is facing red alliance
  public enum reefZone {
    AB(180),
    CD(120),
    EF(60),
    GH(0),
    IJ(300),
    KL(240);

    private double angle;

    reefZone(double angle) {
      this.angle = angle;
    }

    private double getRads() {
      return Units.degreesToRadians(angle);
    }
  }

  private static RobotState instance = null;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public Pose2d reefGoalPose;

  private RobotState() {
    reefGoalPose = new Pose2d();
  }

  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  public void setEstimatedPose(Pose2d pose) {
    estimatedPose = pose;
  }

  public Pose2d getReefGoalPose(boolean clockwise) {
    List<Pose2d> reefGoals = new ArrayList<Pose2d>();
    for (reefZone zone : reefZone.values())
      reefGoals.add(
          new Pose2d(
              FieldConstants.reef.plus(
                  new Translation2d(1.4, 0.2 * (clockwise ? -1 : 1))
                      .rotateBy(new Rotation2d(zone.getRads()))),
              new Rotation2d(zone.getRads() + Math.PI)));
    Pose2d[] reefGoalsArray = new Pose2d[reefGoals.size()];
    reefGoalsArray = reefGoals.toArray(reefGoalsArray);
    Logger.recordOutput("RobotState/reefPositions", reefGoalsArray);
    reefGoalPose = estimatedPose.nearest(reefGoals);
    Logger.recordOutput("RobotState/goalPose", reefGoalPose);
    return estimatedPose.nearest(reefGoals);
  }

  public boolean atGoal() {
    return reefGoalPose.minus(estimatedPose).getTranslation().getNorm() < 0.1;
  }

  // @Override
  // public void periodic() {
  //   this.getReefGoalPose();
  // }
}
