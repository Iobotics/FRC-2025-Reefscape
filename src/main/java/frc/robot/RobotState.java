package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  // 0 degrees is facing red alliance
  public enum reefZone {
    AB(180),
    CD(240),
    EF(300),
    GH(0),
    IJ(60),
    KL(120);

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

      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return new Pose2d(
            FieldConstants.center
                .plus(FieldConstants.centerToReef.times(1))
                .plus(
                    new Translation2d(offset, clockwise ? REEF_CW_OFFSET : REEF_CCW_OFFSET)
                        .rotateBy(this.getAngle())),
            getParallelAngle());
      }
      return new Pose2d(
          FieldConstants.center
              .plus(FieldConstants.centerToReef.times(-1))
              .plus(
                  new Translation2d(offset, clockwise ? REEF_CW_OFFSET : REEF_CCW_OFFSET)
                      .rotateBy(this.getAngle())),
          getParallelAngle());
    }

    /**
     * Get the reef zone's pose for the bot to score coral
     *
     * @param clockwise true will return the pose for the higher letter of the two, AB -> B, false
     *     will return the lower letter, AB -> A
     * @return the pose object representing the reef zone
     */
    private Pose2d getPose(boolean clockwise) {
      return getPose(clockwise, REEF_GOAL_OFFSET);
    }

    /**
     * Get the reef zone's center pose for the bot to remove algae
     *
     * @param offset
     * @return
     */
    public Pose2d getCenterPose(double offset) {
      return new Pose2d(
          FieldConstants.center
              .plus(
                  FieldConstants.centerToReef.times(
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                          ? 1
                          : -1))
              .plus(new Translation2d(offset, 0).rotateBy(this.getAngle())),
          getParallelAngle());
    }

    /**
     * Get the reef zone's center pose for the bot to remove algae
     *
     * @return the pose object representing the reef zone
     */
    public Pose2d getCenterPose() {
      return getCenterPose(REEF_GOAL_OFFSET);
    }

    /**
     * get if the algae is high on the reef zone
     *
     * @return true if the algae is high
     */
    public boolean isAlgaeHigh() {
      return this == reefZone.AB || this == reefZone.EF || this == reefZone.IJ;
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
      return Units.degreesToRadians(angle)
          + (DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
              ? 0
              : Math.PI);
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
  private static final double REEF_CW_OFFSET = -0.15;
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
                  new Translation2d(1.28, REEF_CW_OFFSET).rotateBy(new Rotation2d(zone.getRads()))),
              new Rotation2d(zone.getRads() + Math.PI)));
      reefGoalsCCW.add(
          new Pose2d(
              reef.plus(
                  new Translation2d(1.28, REEF_CCW_OFFSET)
                      .rotateBy(new Rotation2d(zone.getRads()))),
              new Rotation2d(zone.getRads() + Math.PI)));
    }

    Optional<Pose3d> IJPose = VisionConstants.aprilTagLayout.getTagPose(6);

    Pose2d IJPose2 = new Pose2d(IJPose.get().getX(), IJPose.get().getZ(), new Rotation2d());

    Pose2d offset =
        new Pose2d(Units.inchesToMeters(6.5), 0., new Rotation2d())
            .rotateBy(new Rotation2d(Units.degreesToRadians(60)));

    IJPose2.plus(new Transform2d(offset.getTranslation(), offset.getRotation()));

    Logger.recordOutput("pose", IJPose2);
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

  public void setSelectedSide(reefZone side, boolean clockwise, Consumer<Pose2d> callback) {
    selectedSide = side;
    selectedDirectionClockwise = clockwise;
    callback.accept(getSelectedSidePose());
  }

  /**
   * Get the reef zone's pose for the bot to score
   *
   * @return the pose object representing the reef zone
   */
  public Pose2d getSelectedSidePose() {
    return selectedSide.getPose(selectedDirectionClockwise);
  }

  /**
   * Get the reef zone's pose for the bot to score
   *
   * @param offset the distance from the center of the reef structure to the goal, the default for
   *     the bot is 1.28 (up against the reef)
   * @return
   */
  public Pose2d getSelectedSidePose(double offset) {
    return selectedSide.getPose(selectedDirectionClockwise, offset);
  }

  /**
   * Get the reef zone's center pose for the bot to remove algae
   *
   * @return the pose object representing the reef zone
   */
  public Pose2d getSelectedSideAlgaePose(double offset) {
    return selectedSide.getCenterPose(offset);
  }

  public boolean isSelectedSideAlgaeHigh() {
    return selectedSide.isAlgaeHigh();
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

  public Rotation2d getSelectedSideParallelAngle() {
    return selectedSide.getParallelAngle();
  }

  public double getStationAngle(Supplier<Pose2d> pose) {
    if (pose.get().getY() > 4.0) {
      return -(DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
          ? 126
          : 54);
    } else {
      return DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
          ? 126
          : 54;
    }
  }
}
