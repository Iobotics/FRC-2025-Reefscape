package frc.robot;

import static edu.wpi.first.math.util.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

  public Pose2d getReefGoalPose(Pose2d robotPose, boolean clockwise) {
    setEstimatedPose(robotPose);
    List<Pose2d> reefGoals = new ArrayList<Pose2d>();
    Translation2d reef =
        FieldConstants.center.plus(
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? FieldConstants.centerToReef
                : FieldConstants.centerToReef.times(-1));
    for (reefZone zone : reefZone.values())
      reefGoals.add(
          new Pose2d(
              reef.plus(
                  new Translation2d(1.2, 0.138 * (clockwise ? -1 : 1))
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

  public Pose2d getStationGoalPose() {
    List<Pose2d> stationGoals = new ArrayList<Pose2d>();
    Transform2d offset =
        new Transform2d(new Translation2d(-0.5, 0.0), new Rotation2d(Units.degreesToRadians(0)));
    var stations =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
            ? FieldConstants.blueStations
            : FieldConstants.redStations;
    stationGoals.add(stations.get(0).plus(offset));
    stationGoals.add(stations.get(1).plus(offset));
    Pose2d[] loggedStationGoals = new Pose2d[stationGoals.size()];
    loggedStationGoals = stationGoals.toArray(loggedStationGoals);
    Logger.recordOutput("stationPositions", loggedStationGoals);
    return estimatedPose.nearest(stationGoals);
  }
}
