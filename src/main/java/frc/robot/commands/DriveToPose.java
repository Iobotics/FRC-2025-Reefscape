/**
 * drive to pose command, uses single pid controller for x and y to prevent curving to target
 * instead calculates vector and uses that to drive straight to target
 */
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private Drive drive;
  private Supplier<Pose2d> target;
  private Pose2d currentPose;

  private boolean tuning = true;

  private static final LoggedTunableNumber driveP =
      new LoggedTunableNumber("DriveToPose/Drive/kP", 7.0);
  private static final LoggedTunableNumber driveI =
      new LoggedTunableNumber("DriveToPose/Drive/kI", 0);
  private static final LoggedTunableNumber driveD =
      new LoggedTunableNumber("DriveToPose/Drive/kD", 0);

  private static final LoggedTunableNumber thetaP =
      new LoggedTunableNumber("DriveToPose/Theta/kP", 2.0);
  private static final LoggedTunableNumber thetaI =
      new LoggedTunableNumber("DriveToPose/Theta/kI", 0);
  private static final LoggedTunableNumber thetaD =
      new LoggedTunableNumber("DriveToPose/Theta/kD", 0);

  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/Drive/Tolerance", 0.05);
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/Theta/Tolerance", Units.degreesToRadians(3.5));

  private static final LoggedTunableNumber driveMaxSpeed =
      new LoggedTunableNumber("DriveToPose/Drive/MaxSpeed", 1.0);
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/Drive/MaxAcceleration", 3.0);

  private static final LoggedTunableNumber thetaMaxSpeed =
      new LoggedTunableNumber("DriveToPose/Theta/maxSpeed", 1);
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/Theta/maxAcceleration", 1.5);

  private final PIDController driveController =
      new PIDController(driveP.get(), driveI.get(), driveD.get());

  private TrapezoidProfile driveProfile =
      new TrapezoidProfile(new Constraints(driveMaxSpeed.get(), driveMaxAcceleration.get()));

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaP.get(),
          thetaI.get(),
          thetaD.get(),
          new Constraints(thetaMaxSpeed.get(), thetaMaxAcceleration.get()),
          Constants.loopPeriodSecs);

  private State currentState = new State();

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    driveController.setTolerance(driveTolerance.get());
    thetaController.setTolerance(thetaTolerance.get());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.drive = drive;
    this.target = target;
    addRequirements(drive);
  }

  public DriveToPose(
      Drive drive, Supplier<Pose2d> target, double driveMaxSpeed, double driveMaxAcceleration) {
    this(drive, target);
    tuning = false;
    driveProfile = new TrapezoidProfile(new Constraints(driveMaxSpeed, driveMaxAcceleration));
  }

  @Override
  public void initialize() {
    currentPose = drive.getPose();
    currentState =
        new State(target.get().getTranslation().minus(currentPose.getTranslation()).getNorm(), 0);
    thetaController.reset(currentPose.getRotation().getRadians());
    driveController.reset();
  }

  @Override
  public void execute() {
    if (tuning) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () -> driveController.setPID(driveP.get(), driveI.get(), driveD.get()),
          driveP,
          driveI,
          driveD);
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () -> thetaController.setPID(thetaP.get(), thetaI.get(), thetaD.get()),
          thetaP,
          thetaI,
          thetaD);
      LoggedTunableNumber.ifChanged(
          hashCode(), () -> driveController.setTolerance(driveTolerance.get()), driveTolerance);
      LoggedTunableNumber.ifChanged(
          hashCode(), () -> thetaController.setTolerance(thetaTolerance.get()), thetaTolerance);
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () ->
              driveProfile =
                  new TrapezoidProfile(
                      new Constraints(driveMaxSpeed.get(), driveMaxAcceleration.get())),
          driveMaxSpeed,
          driveMaxAcceleration);
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () ->
              thetaController.setConstraints(
                  new Constraints(thetaMaxSpeed.get(), thetaMaxAcceleration.get())),
          thetaMaxSpeed,
          thetaMaxAcceleration);
    }

    Translation2d direction = target.get().getTranslation().minus(currentPose.getTranslation());
    currentState = driveProfile.calculate(Constants.loopPeriodSecs, currentState, new State());

    Logger.recordOutput("DriveToPose/direction", direction);
    Logger.recordOutput("DriveToPose/target", target.get());
    Logger.recordOutput("DriveToPose/angle", direction.getAngle());
    Logger.recordOutput("DriveToPose/currentState", currentState.position);
    Logger.recordOutput(
        "DriveToPose/currentPose",
        target.get().getTranslation().minus(currentPose.getTranslation()).getNorm());

    currentPose = drive.getPose();

    double driveSpeed =
        -driveController.calculate(
            target.get().getTranslation().minus(currentPose.getTranslation()).getNorm(),
            currentState.position);

    Logger.recordOutput("DriveToPose/driveSpeed", driveSpeed);

    Translation2d driveVelocity =
        new Translation2d(
            driveSpeed * direction.getX() / direction.getNorm(),
            driveSpeed * direction.getY() / direction.getNorm());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                driveVelocity.getX(),
                driveVelocity.getY(),
                thetaController.calculate(
                    currentPose.getRotation().getRadians(),
                    target.get().getRotation().getRadians())),
            currentPose.getRotation()));
    // driveController.calculate(currentPose.getTranslation().getNorm(), currentState.position);
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return EqualsUtil.epsilonEquals(
            target.get().getTranslation().minus(currentPose.getTranslation()).getNorm(),
            0,
            driveTolerance.get())
        && thetaController.atGoal();
  }
}
