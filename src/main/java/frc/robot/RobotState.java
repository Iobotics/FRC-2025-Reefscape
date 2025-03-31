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

    private reefZone next() {
      return reefZone.values()[(this.ordinal() + 1) % reefZone.values().length];
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

  private reefZone selectedSide = reefZone.AB;

  public Pose2d reefGoalPose;
  private List<Pose2d> reefGoalsCW;
  private List<Pose2d> reefGoalsCCW;

  private RobotState() {
    reefGoalPose = new Pose2d();
    reefGoalsCW = new ArrayList<Pose2d>();
    reefGoalsCCW = new ArrayList<Pose2d>();

    Translation2d reef =
        FieldConstants.center.plus(
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? FieldConstants.centerToReef
                : FieldConstants.centerToReef.times(-1));
    for (reefZone zone : reefZone.values()) {
      reefGoalsCW.add(
          new Pose2d(
              reef.plus(
                  new Translation2d(1.28, 0.12 * -1).rotateBy(new Rotation2d(zone.getRads()))),
              new Rotation2d(zone.getRads() + Math.PI)));
      reefGoalsCCW.add(
          new Pose2d(
              reef.plus(new Translation2d(1.28, 0.192).rotateBy(new Rotation2d(zone.getRads()))),
              new Rotation2d(zone.getRads() + Math.PI)));
    }
  }

  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  public Pose2d cycleSelectedSide() {
    selectedSide = selectedSide.next();
    return reefGoalsCCW.get(selectedSide.ordinal());
  }

  public Pose2d getSelectedSidePose(boolean clockwise) {
    return clockwise ? reefGoalsCW.get(selectedSide.ordinal()) : reefGoalsCCW.get(selectedSide.ordinal());
  }

  public void setEstimatedPose(Pose2d pose) {
    estimatedPose = pose;
  }

  public Pose2d getReefGoalPose(int index) {
    return reefGoalsCCW.get(index);
  }

  public Pose2d getReefGoalPose(Pose2d robotPose, boolean clockwise) {
    setEstimatedPose(robotPose);
    return estimatedPose.nearest(clockwise ? reefGoalsCW : reefGoalsCCW);
  }

  public void displayReefPose() {
    Logger.recordOutput("reef", reefGoalsCCW.toArray(new Pose2d[reefGoalsCCW.size()]));
    Logger.recordOutput("reefc", reefGoalsCW.toArray(new Pose2d[reefGoalsCW.size()]));
  }

  public boolean atGoal() {
    return reefGoalPose.minus(estimatedPose).getTranslation().getNorm() < 0.1;
  }

  public Pose2d getStationGoalPose() {
    List<Pose2d> stationGoals = new ArrayList<Pose2d>();
    Transform2d offset =
        new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d(Units.degreesToRadians(0)));
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
