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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.Sensor.IntakeSensor;
import frc.robot.subsystems.Sensor.RangeSensor;
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
  private final RangeSensor rangeSensor;
  private final IntakeSensor sensor;
  private final LED LED;
  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandJoystick joystick1 = new CommandJoystick(2);
  private final CommandJoystick joystick2 = new CommandJoystick(3);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        elevator = new Elevator(new ElevatorIOTalonFX());
        arm = new Arm(new ArmIOSparkFlex());

        sensor = new IntakeSensor();
        rangeSensor = new RangeSensor();
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
                    "frontCamera", VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    "funnelCamera", VisionConstants.robotToCamera1, drive::getPose));

        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        CoralManipulator = new CoralManipulator(new CoralManipulatorIO() {});
        sensor = new IntakeSensor();
        LED = new LED();
        rangeSensor = new RangeSensor();
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
        sensor = new IntakeSensor();
        LED = new LED();
        rangeSensor = new RangeSensor();
        break;
    }

    scoreL4 = CoralCommands.scoreCoral(Goal.SCOREL4, elevator, CoralManipulator, arm);
    scoreL3 = CoralCommands.scoreCoral(Goal.SCOREL3, elevator, CoralManipulator, arm);
    scoreL2 = CoralCommands.scoreCoral(Goal.SCOREL2, elevator, CoralManipulator, arm);

    Command raiseL4 =
        Commands.sequence(
            Commands.runOnce(() -> LED.setColor(Color.kYellow)),
            elevator.getSetpointCommand(Goal.SCOREL4).withTimeout(0.6),
            Commands.runOnce(() -> arm.setGoal(Goalposition.SCOREL4)));

    intakeCoral = CoralManipulator.getCommand(sensor, LED);

    NamedCommands.registerCommand("Wait for Coral", CoralManipulator.waitForCoral(sensor));
    NamedCommands.registerCommand("Intake Coral", intakeCoral);
    NamedCommands.registerCommand(
        "L4 score", CoralCommands.scoreL4(elevator, CoralManipulator, arm));
    NamedCommands.registerCommand("Raise L4", raiseL4);
    NamedCommands.registerCommand(
        "L4 Release", CoralCommands.releaseL4(elevator, arm, CoralManipulator).withTimeout(0.6));

    drive.configureAutoBuilder();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
            () ->
                -driveController.getLeftY()
                    * (driveController.rightTrigger(0.2).getAsBoolean() ? 0.8 : 1),
            () ->
                -driveController.getLeftX()
                    * (driveController.rightTrigger(0.2).getAsBoolean() ? 0.8 : 1),
            () ->
                -driveController.getRightX()
                    * (driveController.rightTrigger(0.2).getAsBoolean() ? 0.8 : 1)));

    // UNCOMMENT BELOW FOR JOYSTICK
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive, () -> -joystick1.getY(), () -> -joystick1.getX(), () -> -joystick2.getX()));

    // Reset gyro to 0° when Y button is pressed
    driveController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                new Rotation2d(
                                    (DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red
                                        ? 0
                                        : Math.PI)))),
                    drive)
                .ignoringDisable(true));

    driveController
        .x()
        .onTrue(
            Commands.defer(
                () ->
                    drive.pathfindToPose(
                        RobotState.getInstance().getStationGoalPose(),
                        new Rotation2d(Units.degreesToRadians(54))),
                Set.of(drive)));

    driveController.x().onFalse(Commands.runOnce(() -> drive.getCurrentCommand().cancel()));

    driveController
        .b()
        .onTrue(
            Commands.defer(
                () ->
                    drive.pathfindToPose(
                        RobotState.getInstance().getReefGoalPose(drive.getPose(), false)),
                Set.of(drive)));
    driveController.b().onFalse(Commands.runOnce(() -> drive.getCurrentCommand().cancel()));

    driveController
        .a()
        .onTrue(
            Commands.defer(
                () ->
                    drive.pathfindToPose(
                        RobotState.getInstance().getReefGoalPose(drive.getPose(), true)),
                Set.of(drive)));
    driveController.a().onFalse(Commands.runOnce(() -> drive.getCurrentCommand().cancel()));

    // driveController.leftStick().whileTrue(Commands.runOnce(() -> arm.runVolts(2)));

    // driveController.leftStick().onFalse(Commands.runOnce(() -> arm.runVolts(0)));

    // driveController.x().onTrue(CoralCommands.scoreL4(elevator, CoralManipulator, arm));

    // operatorController
    //     .y()
    //     .whileTrue(
    //         Commands.startEnd(() -> elevator.setGoal(Goal.SCOREL4), () ->
    // elevator.returnToHome()));

    // == arm Controls ==
    operatorController
        .pov(0)
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setGoal(Goalposition.DEFAULT);
                  elevator.setGoal(Goal.STOW);
                }));

    operatorController
        .pov(90)
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setGoal(Goalposition.INTAKEALGAE);
                  elevator.setGoal(Goal.UPPERALGAE);
                }));
    operatorController
        .pov(180)
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setGoal(Goalposition.INTAKEALGAE);
                  elevator.setGoal(Goal.LOWERALGAE);
                }));

    operatorController
        .pov(270)
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setGoal(Goalposition.INTAKEALGAE);
                  elevator.setGoal(Goal.HOLDALGAE);
                }));

    operatorController
        .rightTrigger(0.2)
        .onTrue(Commands.runOnce(() -> elevator.setDistanceOffset(true)));

    operatorController
        .rightTrigger(0.2)
        .onFalse(Commands.runOnce(() -> elevator.setDistanceOffset(false)));

    // operatorController.b().onTrue(CoralCommands.scoreL4(elevator, CoralManipulator, arm));

    // operatorController
    //     .a()
    //     .onTrue(CoralCommands.scoreCoral(Goal.SCOREL3, elevator, CoralManipulator, arm));

    // operatorController
    //     .x()
    //     .onTrue(CoralCommands.scoreCoral(Goal.SCOREL2, elevator, CoralManipulator, arm));

    operatorController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setGoal(Goalposition.SCOREL1);
                  elevator.setGoal(Goal.SCOREL1);
                }));

    operatorController
        .x()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  elevator.setGoal(Goal.SCOREL2);
                  LED.setColor(Color.kYellow);
                },
                () -> {
                  elevator.setGoal(Goal.STOW);
                  LED.setColor(Color.kRed);
                }));

    operatorController
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  elevator.setGoal(Goal.SCOREL3);
                  arm.setGoal(Goalposition.SCOREL3);
                  LED.setColor(Color.kYellow);
                },
                () -> {
                  elevator.setGoal(Goal.STOW);
                  arm.setGoal(Goalposition.DEFAULT);
                  LED.setColor(Color.kRed);
                }));

    operatorController
        .b()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> LED.setColor(Color.kYellow)),
                elevator.getSetpointCommand(Goal.SCOREL4).withTimeout(0.6),
                Commands.runOnce(() -> arm.setGoal(Goalposition.SCOREL4))));

    operatorController
        .b()
        .onFalse(
            Commands.sequence(
                Commands.runOnce(() -> LED.setColor(Color.kRed)),
                Commands.run(() -> arm.setGoal(Goalposition.DEFAULT)).withTimeout(0.1),
                Commands.runOnce(() -> elevator.setGoal(Goal.STOW))));

    operatorController.leftBumper().whileTrue(CoralManipulator.getCommand(sensor, LED));
    operatorController.leftBumper().onFalse(Commands.runOnce(() -> CoralManipulator.setOutake(0)));

    operatorController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> CoralManipulator.setOutake(-0.2), () -> CoralManipulator.setOutake(0)));

    operatorController
        .leftTrigger(0.2)
        .whileTrue(
            Commands.startEnd(
                () -> CoralManipulator.setOutake(1),
                () -> {
                  CoralManipulator.setOutake(0);
                  LED.setColor(Color.kRed);
                }));

    // MANUAL CONTROLS

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
                  arm.setGoal(Goalposition.CUSTOM);
                  elevator.setGoal(Goal.SCOREL4);
                }));

    driveController
        .pov(180)
        .onTrue(
            Commands.sequence(
                Commands.run(() -> arm.setGoal(Goalposition.HOLDALGAE)).withTimeout(0.2),
                Commands.run(() -> CoralManipulator.setOutake(1)).withTimeout(0.8),
                Commands.runOnce(() -> CoralManipulator.setOutake(0))));
    // ONE DRIVE CONTROLS

    // driveController
    //     .pov(90)
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               arm.setGoal(Goalposition.INTAKEALGAE);
    //               elevator.setGoal(Goal.UPPERALGAE);
    //             }));

    // driveController
    //     .pov(180)
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               arm.setGoal(Goalposition.INTAKEALGAE);
    //               elevator.setGoal(Goal.LOWERALGAE);
    //             }));

    // driveController
    //     .pov(270)
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               arm.setGoal(Goalposition.INTAKEALGAE);
    //               elevator.setGoal(Goal.HOLDALGAE);
    //             }));

    // driveController
    //     .x()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> elevator.setGoal(Goal.SCOREL2), () -> elevator.setGoal(Goal.STOW)));

    // driveController
    //     .a()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> elevator.setGoal(Goal.SCOREL3), () -> elevator.setGoal(Goal.STOW)));

    // driveController
    //     .b()
    //     .onTrue(
    //         Commands.sequence(
    //             elevator.getSetpointCommand(Goal.SCOREL4).withTimeout(0.7),
    //             Commands.runOnce(() -> arm.setGoal(Goalposition.SCOREL4))));

    // driveController
    //     .b()
    //     .onFalse(
    //         Commands.sequence(
    //             Commands.run(() -> arm.setGoal(Goalposition.DEFAULT)).withTimeout(0.4),
    //             Commands.runOnce(() -> elevator.setGoal(Goal.STOW))));

    // driveController.leftBumper().whileTrue(CoralManipulator.getCommand(sensor, LED));
    // driveController.leftBumper().onFalse(Commands.runOnce(() -> CoralManipulator.setOutake(0)));

    // driveController
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> CoralManipulator.setOutake(-0.5), () -> CoralManipulator.setOutake(0)));

    // driveController
    //     .leftTrigger(0.2)
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> CoralManipulator.setOutake(1),
    //             () -> {
    //               CoralManipulator.setOutake(0);
    //               LED.setColor(Color.kRed);
    //             }));
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
