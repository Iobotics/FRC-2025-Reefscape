// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = true;
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // motor controller IDs
  public static final int leftIntakeID = 13;
  public static final int rightIntakeID = 14;
  public static final int topLeftOutakeID = 15;
  public static final int topRightOutakeID = 16;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static Mode getRobot() {
    return currentMode;
  }

  public static final class OIConstants {
    public static int kGamepad = 0;
    public static int kJoystick1 = 4;
    public static int kJoystick2 = 2;
    public static int kFight = 3;
    public static int kCandle = 30;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double intakeSpeed = 0.5;
  }

  public static class FieldConstants {
    // assumes all blue alliance
    public static Translation2d center =
        new Translation2d(Units.inchesToMeters(345.44), Units.inchesToMeters(158.515));
    // 4.240788
    public static Translation2d centerToReef = new Translation2d(-4.280788, 0.0);

    public static List<Pose2d> blueStations =
        List.of(
            new Pose2d(1.167, 0.904, new Rotation2d(Units.degreesToRadians(54 + 180))),
            new Pose2d(
                Units.inchesToMeters(33.51),
                Units.inchesToMeters(291.20),
                new Rotation2d(Units.degreesToRadians(306))));

    public static List<Pose2d> redStations =
        List.of(
            new Pose2d(
                Units.inchesToMeters(657.37),
                Units.inchesToMeters(25.80),
                new Rotation2d(Units.degreesToRadians(126 + 180))),
            new Pose2d(
                Units.inchesToMeters(657.37),
                Units.inchesToMeters(291.20),
                new Rotation2d(Units.degreesToRadians(234 + 180))));
  }
}
