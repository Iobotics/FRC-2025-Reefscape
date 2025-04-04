package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.CoralManipulator.CoralManipulator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public final class AutoScore {

  private AutoScore() {}

  public static Command checkDistance(Drive drive) {
    double distance =
        drive
            .getPose()
            .getTranslation()
            .minus(RobotState.getInstance().getSelectedSidePose().getTranslation())
            .getNorm();
    Logger.recordOutput("Distance to target", distance);
    if (Math.abs(distance) > 0.3) {
      return drive.pathFindToPose(RobotState.getInstance().getSelectedSidePose(2.0));
    }
    return Commands.none();
  }

  public static Command checkDistance(Drive drive, Pose2d pose) {
    double distance =
        drive
            .getPose()
            .getTranslation()
            .minus(RobotState.getInstance().getSelectedSidePose().getTranslation())
            .getNorm();
    Logger.recordOutput("Distance to target", distance);
    if (Math.abs(distance) > 0.4) {
      return drive.pathFindToPose(pose);
    }
    return Commands.none();
  }

  public static Command autoScoreL4(
      Drive drive, Elevator elevator, Arm arm, CoralManipulator coralManipulator) {
    var command =
        Commands.defer(
            () ->
                Commands.sequence(
                    checkDistance(drive),
                    Commands.parallel(
                        new DriveToPose(
                            drive, () -> RobotState.getInstance().getSelectedSidePose(), 0.8, 3),
                        CoralCommands.raiseL4(elevator, arm)),
                    new WaitCommand(0.1),
                    CoralCommands.releaseL4(elevator, arm, coralManipulator).withTimeout(1.1),
                    new WaitCommand(0.1)),
            Set.of(drive, elevator, arm, coralManipulator));
    // command.addRequirements(drive, elevator, arm, coralManipulator);
    return command;
  }

  public static Command autoScoreL3(
      Drive drive, Elevator elevator, Arm arm, CoralManipulator coralManipulator) {
    var command =
        Commands.defer(
            () ->
                Commands.sequence(
                    checkDistance(drive),
                    Commands.parallel(
                        new DriveToPose(
                            drive, () -> RobotState.getInstance().getSelectedSidePose(1.28), 1, 3),
                        CoralCommands.raiseL3(elevator, arm)),
                    new WaitCommand(0.1),
                    CoralCommands.releaseL3(elevator, arm, coralManipulator).withTimeout(0.6)),
            Set.of(drive, elevator, arm, coralManipulator));
    command.addRequirements(drive, elevator, arm, coralManipulator);
    return command;
  }

  public static Command autoScoreL2(
      Drive drive, Elevator elevator, Arm arm, CoralManipulator coralManipulator) {
    var command =
        Commands.defer(
            () ->
                Commands.sequence(
                    checkDistance(drive),
                    Commands.parallel(
                        new DriveToPose(
                            drive, () -> RobotState.getInstance().getSelectedSidePose(1.28), 1, 3),
                        CoralCommands.raiseL2(elevator)),
                    CoralCommands.releaseL2(elevator, coralManipulator).withTimeout(0.6)),
            Set.of(drive, elevator, arm, coralManipulator));
    command.addRequirements(drive, elevator, arm, coralManipulator);
    return command;
  }

  public static Command removeAlgae(
      Drive drive, Elevator elevator, Arm arm, CoralManipulator coralManipulator) {
    var command =
        Commands.defer(
            () ->
                Commands.sequence(
                    checkDistance(drive, RobotState.getInstance().getSelectedSideAlgaePose(1.7)),
                    new DriveToPose(
                        drive,
                        () -> RobotState.getInstance().getSelectedSideAlgaePose(1.7),
                        0.8,
                        1),
                    Commands.parallel(
                        new DriveToPose(
                            drive,
                            () -> RobotState.getInstance().getSelectedSideAlgaePose(1.26),
                            0.7,
                            1),
                        RobotState.getInstance().isSelectedSideAlgaeHigh()
                            ? CoralCommands.removeHighAlgae(elevator, arm, coralManipulator)
                            : CoralCommands.removeLowAlgae(elevator, arm, coralManipulator)),
                    new WaitCommand(0.2),
                    new DriveToPose(
                        drive,
                        () -> RobotState.getInstance().getSelectedSideAlgaePose(1.7),
                        0.7,
                        1),
                    CoralCommands.holdAlgae(elevator, arm, coralManipulator).withTimeout(0.4)),
            Set.of(drive, elevator, arm, coralManipulator));
    return command;
  }
}
