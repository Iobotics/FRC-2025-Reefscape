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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.Goalposition;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.Arm.ArmIOSim;
import frc.robot.subsystems.Arm.ArmIOSparkFlex;
import frc.robot.subsystems.CoralManipulator.CoralManipulator;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorIO;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorIOSpark;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.Sensor.Sensor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Goal;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Arm arm;
  private final CoralManipulator CoralManipulator;
  private final Sensor sensor;
  private final LED LED;
  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController operatorController2 =
      new CommandXboxController(2); // temporary
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Trigger scoreTrigger;
  private final Trigger inZoneTrigger;

  public Command scoreL4;
  public Command scoreL3;
  public Command scoreL2;
  public Command intakeCoral;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("frontCamera", VisionConstants.robotToCamera0),
                new VisionIOPhotonVision("funnelCamera", VisionConstants.robotToCamera1));
        elevator = new Elevator(new ElevatorIOTalonFX());
        arm = new Arm(new ArmIOSparkFlex());

        sensor = new Sensor();
        LED = new LED();
        CoralManipulator = new CoralManipulator(new CoralManipulatorIOSpark());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "camera", VisionConstants.robotToCamera0, drive::getPose));

        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        CoralManipulator = new CoralManipulator(new CoralManipulatorIO() {});
        sensor = new Sensor();
        LED = new LED();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIO() {});
        CoralManipulator = new CoralManipulator(new CoralManipulatorIO() {});
        sensor = new Sensor();
        LED = new LED();
        break;
    }

    scoreTrigger = new Trigger(elevator::atGoal);
    inZoneTrigger = new Trigger(RobotState.getInstance()::atGoal);

    scoreL4 = CoralCommands.scoreCoral(Goal.SCOREL4, elevator, CoralManipulator, arm);
    scoreL3 = CoralCommands.scoreCoral(Goal.SCOREL3, elevator, CoralManipulator, arm);
    scoreL2 = CoralCommands.scoreCoral(Goal.SCOREL2, elevator, CoralManipulator, arm);

    intakeCoral = CoralManipulator.getCommand(sensor, LED).withTimeout(0.4);

    NamedCommands.registerCommand("Intake Coral", intakeCoral);
    NamedCommands.registerCommand("L4 score", Commands.waitSeconds(0));

    drive.configureAutoBuilder();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    // Reset gyro to 0° when B button is pressed
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // driveController
    //     .x()
    //     .onTrue(
    //         Commands.defer(
    //             () -> drive.pathfindToPose(RobotState.getInstance().getStationGoalPose()),
    //             Set.of(drive)));

    // driveController.a().onFalse(Commands.runOnce(() -> drive.getCurrentCommand().cancel()));

    driveController
        .y()
        .onTrue(
            Commands.defer(
                () ->
                    drive.pathfindToPose(
                        RobotState.getInstance().getReefGoalPose(drive.getPose(), false)),
                Set.of(drive)));
    driveController.y().onFalse(Commands.runOnce(() -> drive.getCurrentCommand().cancel()));

    driveController
        .a()
        .whileTrue(
            Commands.startEnd(() -> elevator.setGoal(Goal.CUSTOM), () -> elevator.returnToHome()));

    // driveController.x().onTrue(CoralCommands.scoreL4(elevator, CoralManipulator, arm));

    // operatorController
    //     .y()
    //     .whileTrue(
    //         Commands.startEnd(() -> elevator.setGoal(Goal.SCOREL4), () ->
    // elevator.returnToHome()));

    // == arm Controls ==
    driveController
        .pov(0)
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setGoal(Goalposition.DEFAULT);
                  elevator.setGoal(Goal.STOW);
                }));

    driveController
        .pov(90)
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setGoal(Goalposition.INTAKEALGAE);
                  elevator.setGoal(Goal.UPPERALGAE);
                }));
    driveController
        .pov(180)
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setGoal(Goalposition.HOLDALGAE);
                  elevator.setGoal(Goal.HOLDALGAE);
                }));

    driveController.pov(270).onTrue(CoralCommands.scoreL4(elevator, CoralManipulator, arm));

    driveController.leftBumper().whileTrue(CoralManipulator.getCommand(sensor, LED));
    driveController.leftBumper().onFalse(Commands.runOnce(() -> CoralManipulator.setOutake(0)));

    driveController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> CoralManipulator.setOutake(-0.5), () -> CoralManipulator.setOutake(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
