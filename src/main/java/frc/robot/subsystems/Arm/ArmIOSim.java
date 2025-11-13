package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class ArmIOSim implements ArmIO {
  // Mechanism setup for AdvantageKit
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(1.5, 1.5);
  private final LoggedMechanismLigament2d base;
  private final LoggedMechanismLigament2d shoulder;
  private final LoggedMechanismLigament2d arm;

  // Sim state
  private double armAngleRad = Units.degreesToRadians(30.0);
  private double armVelocityRadPerSec = 0.0;
  private double appliedVolts = 0.0;

  public ArmIOSim() {
    // Root: pivot point at bottom-center of visualization
    var root = mech2d.getRoot("pivot", 0.75, 0.1);

    // Base (static for visual reference)
    base = new LoggedMechanismLigament2d("base", 0.2, 90, 8, new Color8Bit(Color.kGray));

    // Shoulder pivot (rotates with arm)
    shoulder =
        new LoggedMechanismLigament2d("shoulder", 0.3, 90, 6, new Color8Bit(Color.kSteelBlue));

    // Main arm segment (moves with the simulated angle)
    arm = new LoggedMechanismLigament2d("arm", 0.6, 90, 6, new Color8Bit(Color.kAliceBlue));

    root.append(base);
    base.append(shoulder);
    shoulder.append(arm);

    // Log initial mechanism to AdvantageKit
    Logger.recordOutput("Arm/Mechanism2d", mech2d);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Simple physics model
    armVelocityRadPerSec += (appliedVolts * 0.05 - armVelocityRadPerSec * 0.02);
    armAngleRad += armVelocityRadPerSec * Constants.loopPeriodSecs;

    // Clamp to range 0–100 degrees
    armAngleRad = Math.max(0.0, Math.min(Units.degreesToRadians(100), armAngleRad));

    // Update visual ligaments
    shoulder.setAngle(Units.radiansToDegrees(armAngleRad));
    arm.setAngle(Units.radiansToDegrees(armAngleRad));

    // Record to AdvantageKit logger
    inputs.positionRad = armAngleRad;
    inputs.velocityRadPerSec = armVelocityRadPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(appliedVolts) * 0.5;
    inputs.supplyCurrentAmps = inputs.currentAmps;

    Logger.recordOutput("Arm/Mechanism2d", mech2d);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
