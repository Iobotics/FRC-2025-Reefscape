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

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmConstants {

  public static final Rotation2d positionTolerance = Rotation2d.fromDegrees(3.0);
  public static final Translation2d armOrigin = new Translation2d(-0.238, 0.298);
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(360);
  public static final int ArmCANID = 12;
  public static final int currentLimit = 40;
  public static final double GearRatio = 25.0;

  public static final double Armlength = Units.inchesToMeters(10.0);
  public static final Gains gains =
      switch (Constants.getRobot()) {
        case SIM -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        default -> new Gains(1.51, 0.0, 0.0, 0.7, 0.0, 0.0, -0.46);
      };
  // public static TrapezoidProfile.Constraints profileConstraints =
  //     new TrapezoidProfile.Constraints(2 * Math.PI, 15);

  // for tuning
  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(20, 80);

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
