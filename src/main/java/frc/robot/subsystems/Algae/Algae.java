package frc.robot.subsystems.Algae;

import static frc.robot.subsystems.Algae.AlgaeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", gains.kP());

  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Arm/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Arm/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Arm/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Arm/Gains/kG", gains.ffkG());
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber("Arm/LowerLimitDegrees", minAngle.getDegrees());
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber("Arm/UpperLimitDegrees", maxAngle.getDegrees());

  public enum Goal {
    DEFAULT(() -> 0),
    UNJAM_INTAKE(new LoggedTunableNumber("Arm/UnjamDegrees", 40.0)),
    STATION_INTAKE(new LoggedTunableNumber("Arm/StationIntakeDegrees", 45.0)),
    CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 20.0));

    private final DoubleSupplier armSetpointSupplier;

    private Goal(DoubleSupplier armSetpointSupplier) {
      this.armSetpointSupplier = armSetpointSupplier;
    }

    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }


  @AutoLogOutput private Goal goal = Goal.DEFAULT;

  public void setGoal(Goal newGoal) {
    goal = newGoal;

  }
    
  @AutoLogOutput private double currentCompensation = 0.0;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private double goalAngle;
  private ArmFeedforward ff;

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;
  private BooleanSupplier halfStowSupplier = () -> true;
  private boolean brakeModeEnabled = true;

  private boolean wasNotAuto = false;

  private boolean characterizing = false;

  private final AlgaeIO io;

  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  public Algae(AlgaeIO io) {
    this.io = io;
   profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    io.setPID(kP.get(), kI.get(), kD.get());
    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  
  LoggedTunableNumber.ifChanged(
    hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
LoggedTunableNumber.ifChanged(
    hashCode(),
    () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
    kS,
    kG,
    kV,
    kA);
  

    if (disableSupplier.getAsBoolean()
    || (Constants.getRobot() == Constants.Mode.SIM
        && DriverStation.isAutonomousEnabled()
        && wasNotAuto)) {
  io.stop();

  setpointState = new TrapezoidProfile.State(inputs.positionRad, 0);
         } wasNotAuto = !DriverStation.isAutonomousEnabled();
  setBrakeMode(!coastSupplier.getAsBoolean());
  if (!characterizing && brakeModeEnabled && !disableSupplier.getAsBoolean()) {
    // Run closed loop
    goalAngle =
        goal.getRads();
    setpointState =
        profile.calculate(
            Constants.loopPeriodSecs,
            setpointState,
            new TrapezoidProfile.State(
                MathUtil.clamp(
                    goalAngle,
                    Units.degreesToRadians(lowerLimitDegrees.get()),
                    Units.degreesToRadians(upperLimitDegrees.get())),
                0.0));
    if (goal == Goal.DEFAULT
        && EqualsUtil.epsilonEquals(goalAngle, minAngle.getRadians())
        && atGoal()) {
      io.stop();
    } else {
      io.runSetpoint(
          setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));
    
    }
  }
}
public void stop() {
  io.stop();
}

@AutoLogOutput(key = "Arm/AtGoal")
public boolean atGoal() {
  return EqualsUtil.epsilonEquals(setpointState.position, goalAngle, 1e-3);
}

public void setBrakeMode(boolean enabled) {
  if (brakeModeEnabled == enabled) return;
  brakeModeEnabled = enabled;
  io.setBrakeMode(brakeModeEnabled);
}

public void setProfileConstraints(TrapezoidProfile.Constraints constraints) {
  if (EqualsUtil.epsilonEquals(currentConstraints.maxVelocity, constraints.maxVelocity)
      && EqualsUtil.epsilonEquals(currentConstraints.maxAcceleration, constraints.maxVelocity))
    return;
  currentConstraints = constraints;
  profile = new TrapezoidProfile(currentConstraints);
}

public void runCharacterization(double amps) {
  characterizing = true;
  io.runCurrent(amps);
}

public double getCharacterizationVelocity() {
  return inputs.velocityRadsPerSec;
}

public void endCharacterization() {
  characterizing = false;
}

  //--------------------------------------------------------------
  /* 
  public void runSetpoint(double percentVolts) {
    io.setVoltage(percentVolts * 12);
  }
*/
  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  
}
}
