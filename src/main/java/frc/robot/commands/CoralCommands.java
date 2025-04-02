package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.Goalposition;
import frc.robot.subsystems.CoralManipulator.CoralManipulator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Goal;

public final class CoralCommands {
  private CoralCommands() {}

  public static Command scoreCoral(
      Elevator.Goal goal, Elevator elevator, CoralManipulator coralManipulator, Arm arm) {
    return Commands.sequence(
        elevator.getSetpointCommand(goal),
        Commands.run(() -> coralManipulator.setOutake(0.8), coralManipulator).withTimeout(0.4),
        Commands.runOnce(() -> coralManipulator.setOutake(0), coralManipulator),
        elevator.getSetpointCommand(Goal.STOW).withTimeout(2));
  }

  /** elevator auto moves */
  public static Command scoreL4(Elevator elevator, CoralManipulator coralManipulator, Arm arm) {
    return Commands.sequence(
        Commands.sequence(
            elevator.getSetpointCommand(Goal.SCOREL4).withTimeout(0.7),
            Commands.run(() -> arm.setGoal(Goalposition.SCOREL4)).withTimeout(0.5)),
        Commands.run(() -> coralManipulator.setOutake(1.0), coralManipulator).withTimeout(0.5),
        Commands.parallel(
            Commands.run(() -> arm.setGoal(Goalposition.DEFAULT)).withTimeout(0.4),
            Commands.runOnce(() -> coralManipulator.setOutake(0), coralManipulator)),
        elevator.getSetpointCommand(Goal.STOW).withTimeout(0.7));
  }

  public static Command releaseL4(Elevator elevator, Arm arm, CoralManipulator coralManipulator) {
    return Commands.sequence(
        Commands.run(() -> coralManipulator.setOutake(1.0), coralManipulator).withTimeout(0.1),
        Commands.run(() -> arm.setGoal(Goalposition.DEFAULT)).withTimeout(0.3),
        elevator.getSetpointCommand(Goal.STOW).withTimeout(0.2),
        Commands.runOnce(() -> coralManipulator.setOutake(0), coralManipulator));
  }

  public static Command releaseL3(Elevator elevator, Arm arm, CoralManipulator coralManipulator) {
    return Commands.sequence(
        Commands.run(() -> coralManipulator.setOutake(1.0), coralManipulator).withTimeout(0.1),
        Commands.runOnce(() -> arm.setGoal(Goalposition.DEFAULT)),
        elevator.getSetpointCommand(Goal.STOW).withTimeout(0.2),
        Commands.runOnce(() -> coralManipulator.setOutake(0), coralManipulator));
  }

  public static Command releaseL2(Elevator elevator, CoralManipulator coralManipulator) {
    return Commands.sequence(
        Commands.run(() -> coralManipulator.setOutake(1.0), coralManipulator).withTimeout(0.1),
        elevator.getSetpointCommand(Goal.STOW).withTimeout(0.2),
        Commands.runOnce(() -> coralManipulator.setOutake(0), coralManipulator));
  }

  /** doesnt lift elevator */
  public static Command scoreL4(CoralManipulator coralManipulator, Arm arm) {
    return Commands.sequence(
        arm.getSetpointCommand(Goalposition.SCOREL4).withTimeout(0.5),
        Commands.parallel(
            Commands.run(() -> coralManipulator.setOutake(0.8), coralManipulator).withTimeout(0.5),
            arm.getSetpointCommand(Goalposition.DEFAULT).withTimeout(0.5)),
        Commands.runOnce(() -> coralManipulator.setOutake(0), coralManipulator));
  }

  public static Command intakeUpperAlgae(
      Elevator elevator, CoralManipulator coralManipulator, Arm arm) {
    return new Command() {
      @Override
      public void initialize() {
        elevator.setGoal(Goal.UPPERALGAE);
        arm.setGoal(Goalposition.INTAKEALGAE);
        coralManipulator.setOutake(-0.5);
      }

      @Override
      public void end(boolean interrupted) {
        elevator.setGoal(Goal.HOLDALGAE);
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  public static Command raiseL4(Elevator elevator, Arm arm) {
    return Commands.sequence(
        elevator.getSetpointCommand(Goal.SCOREL4).withTimeout(0.6),
        Commands.run(() -> arm.setGoal(Goalposition.SCOREL4)).withTimeout(0.2));
  }

  public static Command raiseL3(Elevator elevator, Arm arm) {
    return Commands.run(
            () -> {
              elevator.setGoal(Goal.SCOREL3);
              arm.setGoal(Goalposition.SCOREL3);
            })
        .withTimeout(0.5);
  }

  public static Command raiseL2(Elevator elevator) {
    return elevator.getSetpointCommand(Goal.SCOREL2).withTimeout(0.2);
  }

  public static Command stow(Elevator elevator, Arm arm) {
    return Commands.sequence(
        Commands.run(() -> arm.setGoal(Goalposition.DEFAULT)).withTimeout(0.2),
        elevator.getSetpointCommand(Goal.STOW).withTimeout(0.2));
  }

  public static Command removeHighAlgae(
      Elevator elevator, Arm arm, CoralManipulator coralManipulator) {
    return Commands.runOnce(
        () -> {
          elevator.setGoal(Goal.UPPERALGAE);
          arm.setGoal(Goalposition.INTAKEALGAE);
          coralManipulator.setOutake(-0.5);
        });
  }

  public static Command removeLowAlgae(
      Elevator elevator, Arm arm, CoralManipulator coralManipulator) {
    return Commands.runOnce(
        () -> {
          elevator.setGoal(Goal.LOWERALGAE);
          arm.setGoal(Goalposition.INTAKEALGAE);
          coralManipulator.setOutake(-0.5);
        });
  }

  public static Command holdAlgae(Elevator elevator, Arm arm, CoralManipulator coralManipulator) {
    return Commands.runOnce(
        () -> {
          elevator.setGoal(Goal.HOLDALGAE);
          arm.setGoal(Goalposition.HOLDALGAE);
          coralManipulator.setOutake(0);
        });
  }

  public static Command releaseAlgae(
      Elevator elevator, Arm arm, CoralManipulator coralManipulator) {
    return Commands.sequence(
        Commands.run(
                () -> {
                  elevator.setGoal(Goal.HOLDALGAE);
                  arm.setGoal(Goalposition.HOLDALGAE);
                  coralManipulator.setOutake(0);
                })
            .withTimeout(0.3),
        Commands.run(() -> coralManipulator.setOutake(0.4)).withTimeout(0.3),
        Commands.runOnce(
            () -> {
              elevator.setGoal(Goal.STOW);
              arm.setGoal(Goalposition.DEFAULT);
              coralManipulator.setOutake(0);
            }));
  }
}
