package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sensor.RangeSensor;
import frc.robot.subsystems.drive.Drive;

public class RangeAlign {

  private RangeAlign() {}

  public static Command get(Drive drive, RangeSensor rangeSensor) {
    return new Command() {

      @Override
      public void initialize() {
      }

      @Override
      public void execute() {
        double left = rangeSensor.isDetected(0) ? rangeSensor.getDistance(0) : 2.0;
        double right = rangeSensor.isDetected(2) ? rangeSensor.getDistance(2) : 2.0;
        if (Math.abs(left-right) < 0.1) {
          drive.runVelocity(new ChassisSpeeds(0,0, 0));
        } else if (left < right) {
          drive.runVelocity(new ChassisSpeeds(0, -0.5, 0));
        } else if (right < left) {
          drive.runVelocity(new ChassisSpeeds(0,0.5, 0));
        }
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }
}
