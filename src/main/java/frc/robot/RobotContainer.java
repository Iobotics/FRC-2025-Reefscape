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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState.reefZone;
import frc.robot.commands.AutoScore;
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
import frc.robot.subsystems.CoralManipulator.CoralManipulatorIOSim;
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
  //   private final CommandJoystick joystick1 = new CommandJoystick(2);
  //   private final CommandJoystick joystick2 = new CommandJoystick(3);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Commands
  public Command scoreL4;
  public Command scoreL3;
  public Command scoreL2;

  public Command removeAlgae;

  public Command intakeCoral;

  public Trigger driveRightStickActive =
      new Trigger(
          () ->
              Math.abs(driveController.getRightX()) > 0.1
                  || Math.abs(driveController.getRightY()) > 0.1);

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
        CoralManipulator = new CoralManipulator(new CoralManipulatorIOSim());
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

    scoreL4 = AutoScore.autoScoreL4(drive, elevator, arm, CoralManipulator);
    scoreL3 = AutoScore.autoScoreL3(drive, elevator, arm, CoralManipulator);
    scoreL2 = AutoScore.autoScoreL2(drive, elevator, arm, CoralManipulator);
    removeAlgae = AutoScore.removeAlgae(drive, elevator, arm, CoralManipulator);

    intakeCoral = CoralManipulator.getCommand(sensor, LED);

    Command scoreL4_F =
        Commands.sequence(
            Commands.runOnce(() -> RobotState.getInstance().setSelectedSide(reefZone.EF, true)),
            scoreL4);

    NamedCommands.registerCommand("Wait for Coral", CoralManipulator.waitForCoral(sensor));
    NamedCommands.registerCommand("Intake Coral", intakeCoral);
    NamedCommands.registerCommand("Score L4-Auto", scoreL4);
    NamedCommands.registerCommand(
        "select-A",
        Commands.runOnce(() -> RobotState.getInstance().setSelectedSide(reefZone.AB, true)));
    NamedCommands.registerCommand(
        "select-B",
        Commands.runOnce(() -> RobotState.getInstance().setSelectedSide(reefZone.AB, false)));
    NamedCommands.registerCommand(
        "select-C",
        Commands.runOnce(() -> RobotState.getInstance().setSelectedSide(reefZone.CD, true)));
    NamedCommands.registerCommand(
        "select-D",
        Commands.runOnce(() -> RobotState.getInstance().setSelectedSide(reefZone.CD, false)));
    NamedCommands.registerCommand(
        "select-E",
        Commands.runOnce(() -> RobotState.getInstance().setSelectedSide(reefZone.EF, true)));
    NamedCommands.registerCommand(
        "select-F",
        Commands.runOnce(() -> RobotState.getInstance().setSelectedSide(reefZone.EF, false)));

    NamedCommands.registerCommand("Remove Algae", removeAlgae);

    drive.configureAutoBuilder();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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

    driveController
        .leftBumper()
        .whileTrue(
            Commands.defer(
                () ->
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driveController.getLeftY(),
                        () -> -driveController.getLeftX(),
                        () -> RobotState.getInstance().getSelectedSideParallelAngle()),
                Set.of(drive)));

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
        .a()
        .onTrue(
            Commands.runOnce(
                () -> drive.displayArbPose(RobotState.getInstance().cycleSelectedSide())));

    driveController
        .x()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                      drive.getCurrentCommand().cancel();
                      try {
                        elevator.getCurrentCommand().cancel();
                      } catch (Exception e) {
                        // Do nothing
                      }
                      try {
                        arm.getCurrentCommand().cancel();
                      } catch (Exception e) {
                        // Do nothing
                      }
                      CoralManipulator.setOutake(0);
                    }),
                CoralCommands.stow(elevator, arm)));

    driveController
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () ->
                    new Rotation2d(
                        Units.degreesToRadians(
                            RobotState.getInstance().getStationAngle(drive::getPose)))));

    operatorController.b().onTrue(AutoScore.autoScoreL4(drive, elevator, arm, CoralManipulator));

    operatorController.a().onTrue(AutoScore.autoScoreL3(drive, elevator, arm, CoralManipulator));

    operatorController.x().onTrue(AutoScore.autoScoreL2(drive, elevator, arm, CoralManipulator));

    operatorController
        .pov(0)
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setGoal(Goalposition.DEFAULT);
                  elevator.setGoal(Goal.STOW);
                }));

    operatorController.pov(90).onTrue(removeAlgae);

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

    operatorController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setGoal(Goalposition.SCOREL1);
                  elevator.setGoal(Goal.SCOREL1);
                }));

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
    // driveController.leftBumper().onFalse(Commands.runOnce(() ->
    // CoralManipulator.setOutake(0)));

    // driveController
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> CoralManipulator.setOutake(-0.5), () ->
    // CoralManipulator.setOutake(0)));

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
