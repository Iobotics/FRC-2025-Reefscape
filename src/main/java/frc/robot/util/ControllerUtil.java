package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ControllerUtil {
    public static Rotation2d snapToReef(double x, double y) {
        double angle = Math.atan2(y, x);
        Logger.recordOutput("preSnap", angle);
        return new Rotation2d(0);
        
        // double angle = Math.atan2(y, x) - (Math.PI / 2) + Math.PI*2;
        // Logger.recordOutput("preSnap", angle);
        // Logger.recordOutput("preSnapDifference", angle % (Math.PI / 3));
        // if (angle % (Math.PI / 3) > (Math.PI / 6)) {
        //     angle -= angle % (Math.PI / 3);
        // } else {
        //     angle += (Math.PI / 3) - (angle % (Math.PI / 3));
        // }
        // Logger.recordOutput("snapToReef", angle);
        // return new Rotation2d(angle);
    }
}
