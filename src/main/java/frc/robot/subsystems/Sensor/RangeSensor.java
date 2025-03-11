package frc.robot.subsystems.Sensor;

import java.io.ObjectInputFilter.Status;
import java.util.List;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PhoenixUtil;

public class RangeSensor {
    CANrange left = new CANrange(33);
    CANrange center = new CANrange(34);
    CANrange right = new CANrange(35);

    List<StatusSignal<Distance>> sensorDistance;
    List<StatusSignal<Boolean>> isDetected;
    List<StatusSignal<Distance>> distanceStdDev;
    
    CANrangeConfiguration sideConfig = new CANrangeConfiguration();
    CANrangeConfiguration centerConfig = new CANrangeConfiguration();
    public RangeSensor() {
        
        PhoenixUtil.tryUntilOk(5, () -> left.getConfigurator().apply(sideConfig));
        PhoenixUtil.tryUntilOk(5, () -> center.getConfigurator().apply(centerConfig));
        PhoenixUtil.tryUntilOk(5, () -> right.getConfigurator().apply(sideConfig));

        sensorDistance = List.of(
            left.getDistance(),
            center.getDistance(),
            right.getDistance()
        );

        isDetected = List.of(
            left.getIsDetected(),
            center.getIsDetected(),
            right.getIsDetected()
        );

        distanceStdDev = List.of(
            left.getDistanceStdDev(),
            center.getDistanceStdDev(),
            right.getDistanceStdDev()
        );

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            Stream.concat(
                sensorDistance.stream(),
                Stream.concat(
                    isDetected.stream(),
                    distanceStdDev.stream()
                )
            ).toArray(StatusSignal[]::new));
    }

    public boolean isDetected(int sensor) {
        return isDetected.get(sensor).getValue();
    }

    public double getDistance() {
        return getDistance(1);
    }

    public double getDistance(int sensor) {
        return sensorDistance.get(sensor).getValueAsDouble();
    }

    /**
     * call every period to update shuffleboard values
     */
    public void updateInputs() {
        Logger.recordOutput("RangeSensor/Distance", 
            sensorDistance.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray());
        Logger.recordOutput("RangeSensor/IsDetected", 
            isDetected.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray());
        Logger.recordOutput("RangeSensor/DistanceStdDev", 
            distanceStdDev.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray());
    }
}
