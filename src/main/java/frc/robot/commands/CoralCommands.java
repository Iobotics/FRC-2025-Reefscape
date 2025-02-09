package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.Goalposition;
import frc.robot.subsystems.CoralManipulator.CoralManipulator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Goal;

public class CoralCommands {
  private CoralCommands() {}

  public static Command scoreCoral(
      Elevator.Goal goal, Elevator elevator, CoralManipulator coralManipulator, Arm arm) {
    return Commands.sequence(
        elevator.getSetpointCommand(goal),
        Commands.run(() -> coralManipulator.setOutake(0.8), coralManipulator).withTimeout(0.15),
        Commands.runOnce(() -> coralManipulator.setOutake(0), coralManipulator),
        elevator.getSetpointCommand(Goal.STOW).withTimeout(2));
  }

  public static Command scoreL4(Elevator elevator, CoralManipulator coralManipulator, Arm arm) {
    return Commands.sequence(
        Commands.parallel(
            elevator.getSetpointCommand(Goal.SCOREL4).withTimeout(2),
            Commands.runOnce(() -> arm.setGoal(Goalposition.SCOREL4), arm)),
        Commands.parallel(
            Commands.run(() -> coralManipulator.setOutake(0.8), coralManipulator).withTimeout(0.5),
            Commands.runOnce(() -> arm.setGoal(Goalposition.DEFAULT), arm)),
        Commands.runOnce(() -> coralManipulator.setOutake(0), coralManipulator),
        elevator.getSetpointCommand(Goal.STOW).withTimeout(2));
  }
}
