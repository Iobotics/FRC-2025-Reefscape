package frc.robot.subsystems.Sensor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;
import java.util.List;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class RangeSensor extends SubsystemBase {
  CANrange left = new CANrange(33);
  CANrange center = new CANrange(34);
  CANrange right = new CANrange(35);

  List<StatusSignal<Distance>> sensorDistance;
  List<StatusSignal<Boolean>> isDetected;
  List<StatusSignal<Distance>> distanceStdDev;

  CANrangeConfiguration sideConfig = new CANrangeConfiguration();
  CANrangeConfiguration centerConfig = new CANrangeConfiguration();

  LoggedTunableNumber centerDistance = new LoggedTunableNumber("RangeSensor/Center/Distance", 0.4);
  LoggedTunableNumber centerX = new LoggedTunableNumber("RangeSensor/Center/XFOV", 27);
  LoggedTunableNumber centerY = new LoggedTunableNumber("RangeSensor/Center/YFOV", 27);

  LoggedTunableNumber sideDistance = new LoggedTunableNumber("RangeSensor/Side/Distance", 0.4);
  LoggedTunableNumber sideX = new LoggedTunableNumber("RangeSensor/Side/XFOV", 27);
  LoggedTunableNumber sideY = new LoggedTunableNumber("RangeSensor/Side/YFOV", 27);

  public RangeSensor() {
    centerConfig.ProximityParams.ProximityThreshold = centerDistance.getAsDouble();
    centerConfig.FovParams.FOVRangeX = centerX.getAsDouble();
    centerConfig.FovParams.FOVRangeY = centerY.getAsDouble();

    sideConfig.ProximityParams.ProximityThreshold = sideDistance.getAsDouble();
    sideConfig.FovParams.FOVRangeX = sideX.getAsDouble();
    sideConfig.FovParams.FOVRangeY = sideY.getAsDouble();

    PhoenixUtil.tryUntilOk(5, () -> left.getConfigurator().apply(sideConfig));
    PhoenixUtil.tryUntilOk(5, () -> center.getConfigurator().apply(centerConfig));
    PhoenixUtil.tryUntilOk(5, () -> right.getConfigurator().apply(sideConfig));

    sensorDistance = List.of(left.getDistance(), center.getDistance(), right.getDistance());

    isDetected = List.of(left.getIsDetected(), center.getIsDetected(), right.getIsDetected());

    distanceStdDev =
        List.of(left.getDistanceStdDev(), center.getDistanceStdDev(), right.getDistanceStdDev());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        Stream.concat(
                sensorDistance.stream(),
                Stream.concat(isDetected.stream(), distanceStdDev.stream()))
            .toArray(StatusSignal[]::new));
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

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          centerConfig.ProximityParams.ProximityThreshold = centerDistance.getAsDouble();
          centerConfig.FovParams.FOVRangeX = centerX.getAsDouble();
          centerConfig.FovParams.FOVRangeY = centerY.getAsDouble();
          PhoenixUtil.tryUntilOk(5, () -> center.getConfigurator().apply(centerConfig));
        },
        centerDistance,
        centerX,
        centerY);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          sideConfig.ProximityParams.ProximityThreshold = sideDistance.getAsDouble();
          sideConfig.FovParams.FOVRangeX = sideX.getAsDouble();
          sideConfig.FovParams.FOVRangeY = sideY.getAsDouble();
          PhoenixUtil.tryUntilOk(5, () -> left.getConfigurator().apply(sideConfig));
          PhoenixUtil.tryUntilOk(5, () -> right.getConfigurator().apply(sideConfig));
        },
        sideDistance,
        sideX,
        sideY);

    BaseStatusSignal.refreshAll(
        Stream.concat(
                sensorDistance.stream(),
                Stream.concat(isDetected.stream(), distanceStdDev.stream()))
            .toArray(StatusSignal[]::new));

    Logger.recordOutput(
        "RangeSensor/Distance",
        sensorDistance.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray());
    Logger.recordOutput(
        "RangeSensor/IsDetected",
        isDetected.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray());
    Logger.recordOutput(
        "RangeSensor/DistanceStdDev",
        distanceStdDev.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray());
  }
}
