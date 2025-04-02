package frc.robot;

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

    /**
     * Get the reef zone's pose for the bot to score
     *
     * @param clockwise true will return the pose for the higher letter of the two, AB -> B, false
     *     will return the lower letter, AB -> A
     * @param offset the distance from the center of the reef structure to the goal, the default for
     *     the bot is 1.28
     * @return the pose object representing the reef zone
     */
    public Pose2d getPose(boolean clockwise, double offset) {

      return new Pose2d(
          FieldConstants.center
              .plus(
                  FieldConstants.centerToReef.times(
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                          ? 1
                          : -1))
              .plus(
                  new Translation2d(offset, clockwise ? REEF_CW_OFFSET : REEF_CCW_OFFSET)
                      .rotateBy(this.getAngle())),
          getParallelAngle());
    }

    /**
     * Get the reef zone's pose for the bot to score
     *
     * @param clockwise true will return the pose for the higher letter of the two, AB -> B, false
     *     will return the lower letter, AB -> A
     * @return the pose object representing the reef zone
     */
    private Pose2d getPose(boolean clockwise) {
      return getPose(clockwise, REEF_GOAL_OFFSET);
    }

    /**
     * Get the angle parallel to the reef zone (what the robot needs to be facing to score)
     *
     * @return the rotation object representing the angle
     */
    private Rotation2d getParallelAngle() {
      return new Rotation2d(getRads() + Math.PI);
    }

    /**
     * Get the next reef zone in the sequence, ab -> cd -> ef -> gh -> ij -> kl -> ab
     *
     * @return the next reef zone
     */
    private reefZone next() {
      return reefZone.values()[(this.ordinal() + 1) % reefZone.values().length];
    }

    /**
     * Get the angle in radians from the center of the reef structure to a specified side
     *
     * @return the angle in radians
     */
    private double getRads() {
      return Units.degreesToRadians(angle);
    }

    /**
     * Get the rotation object representing the angle from the center of the reef structure to a
     * specified side
     *
     * @return the rotation object
     */
    private Rotation2d getAngle() {
      return new Rotation2d(getRads());
    }
  }

  private static RobotState instance = null;

  /**
   * Get the singleton instance of the RobotState, do not instantiate this class directly
   *
   * @return the singleton instance of the RobotState
   */
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private reefZone selectedSide = reefZone.AB;
  private boolean selectedDirectionClockwise = false;

  public Pose2d reefGoalPose;
  private List<Pose2d> reefGoalsCW;
  private List<Pose2d> reefGoalsCCW;

  private static final double REEF_GOAL_OFFSET = 1.28;
  private static final double REEF_CW_OFFSET = -0.12;
  private static final double REEF_CCW_OFFSET = 0.192;

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

  private Pose2d estimatedPose = new Pose2d();

  public Pose2d cycleSelectedSide() {
    if (selectedDirectionClockwise) {
      selectedSide = selectedSide.next();
      selectedDirectionClockwise = false;
    } else {
      selectedDirectionClockwise = true;
    }
    return selectedSide.getPose(selectedDirectionClockwise);
  }

  /**
   * Get the reef zone's pose for the bot to score
   *
   * @return the pose object representing the reef zone
   */
  public Pose2d getSelectedSidePose() {
    return selectedSide.getPose(selectedDirectionClockwise);
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
