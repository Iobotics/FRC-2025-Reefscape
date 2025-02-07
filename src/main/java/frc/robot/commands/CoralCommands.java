package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.Goalposition;
import frc.robot.subsystems.CoralManipulator.CoralManipulator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Goal;

public class CoralCommands {
    private CoralCommands() {
    }

    public static Command scoreCoral(Elevator.Goal goal, Elevator elevator, CoralManipulator coralManipulator, Arm arm) {
        return Commands.sequence(
                elevator.getSetpointCommand(goal).withTimeout(2),
                Commands.run(() -> coralManipulator.setOutake(0.5), coralManipulator)
                    .withTimeout(0.3),
                elevator.getSetpointCommand(Goal.STOW).withTimeout(2));
    }

    
    public static Command scoreL4(Elevator elevator, CoralManipulator coralManipulator, Arm arm) {
        return Commands.sequence(
            Commands.parallel(
                elevator.getSetpointCommand(Goal.SCOREL4).withTimeout(2),
                Commands.runOnce(() -> arm.setGoal(Goalposition.SCOREL4), arm)),
            Commands.parallel(
                Commands.run(() -> coralManipulator.setOutake(0.5), coralManipulator)
                    .withTimeout(0.3),
                Commands.runOnce(() -> arm.setGoal(Goalposition.DEFAULT), arm)
            ),
            elevator.getSetpointCommand(Goal.STOW).withTimeout(2));
    }
}
