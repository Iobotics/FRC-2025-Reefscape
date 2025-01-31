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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CoralFunnel.CoralFunnel;
import frc.robot.subsystems.CoralFunnel.CoralFunnelIO;
import frc.robot.subsystems.CoralFunnel.CoralFunnelIOSim;
import frc.robot.subsystems.CoralManipulator.CoralManipulator;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorIO;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorIOSpark;
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
  private final CoralManipulator CoralManipulator; // SHOULD THIS BE PRIVATE FINAL????
  private final Sensor sensor;
  private final CoralFunnel coralFunnel;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
        // // CHANGE
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("frontCamera", VisionConstants.robotToCamera0));
        elevator = new Elevator(new ElevatorIOTalonFX());

        // CoralManipulator = new CoralManipulator(new CoralManipulatorIO() {});
        sensor = new Sensor();
        coralFunnel = new CoralFunnel(new CoralFunnelIO() {});
        CoralManipulator = new CoralManipulator(new CoralManipulatorIOSpark());
        // sensor = new Sensor();

        // coralFunnel = new CoralFunnel(new CoralFunnelIOSpark());
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
        CoralManipulator = new CoralManipulator(new CoralManipulatorIO() {});
        sensor = new Sensor();
        coralFunnel = new CoralFunnel(new CoralFunnelIOSim());
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
        CoralManipulator = new CoralManipulator(new CoralManipulatorIO() {});
        sensor = new Sensor();
        coralFunnel = new CoralFunnel(new CoralFunnelIO() {});
        break;
    }

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

    SequentialCommandGroup score = new SequentialCommandGroup();

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

    // driveController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // driveController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    // driveController.a().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));

    // driveController.a().onFalse(Commands.run(() -> drive.stop()));

    // driveController.b().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // driveController.b().onFalse(Commands.run(() -> drive.stop()));

    // driveController.x().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    // driveController.x().onFalse(Commands.run(() -> drive.stop()));

    // driveController.y().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // driveController.y().onFalse(Commands.run(() -> drive.stop()));

    // Lock to 0° when A button is held
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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

    Command pathfindingCommand =
        drive.pathfindToPose(new Pose2d(2.8, 4 - 0.1651, Rotation2d.fromDegrees(0)));
    driveController.y().onTrue(pathfindingCommand);
    driveController.y().onFalse(Commands.runOnce(() -> pathfindingCommand.cancel()));

    // RIGHT ON DPAD
    // operatorController
    //     .pov(90)
    //     .whileTrue()

    operatorController
        .a()
        .whileTrue(
            Commands.startEnd(() -> elevator.setGoal(Goal.SCOREL1), () -> elevator.returnToHome()));
    operatorController
        .b()
        .whileTrue(
            Commands.startEnd(() -> elevator.setGoal(Goal.SCOREL2), () -> elevator.returnToHome()));
    operatorController
        .x()
        .whileTrue(
            Commands.startEnd(() -> elevator.setGoal(Goal.SCOREL3), () -> elevator.returnToHome()));
    operatorController
        .y()
        .whileTrue(
            Commands.startEnd(() -> elevator.setGoal(Goal.SCOREL4), () -> elevator.returnToHome()));

    // operatorController
    //     .leftBumper()
    //     .whileTrue(
    //         Commands.startEnd(() -> coralFunnel.runFunnel(0.1), () -> coralFunnel.runFunnel(0)));

    operatorController.leftBumper().whileTrue(CoralManipulator.getCommand(sensor));

    operatorController.leftBumper().onFalse(Commands.runOnce(() -> CoralManipulator.runOutake(0)));

    operatorController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(() -> elevator.manualCurrent(75), () -> elevator.manualCurrent(0)));
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
