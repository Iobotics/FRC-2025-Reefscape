package frc.robot.subsystems.CoralFeeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralFeeder extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkFlex Intake;

  private static SparkClosedLoopController IntakeClosedLoopController;

  private RelativeEncoder IntakeEn;
  private CoralFeederSwitchIO switchIO;
  private final CoralFeederSwitchIOInputsAutoLogged switchInputs =
      new CoralFeederSwitchIOInputsAutoLogged();
  private boolean intakeSwitchState;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxOutput = 1.0;
  private static final double kMinOutput = -1.0;

  public CoralFeeder() {
    Intake = new SparkFlex(CoralFeeder_Constants.intakeId, MotorType.kBrushless);

    IntakeClosedLoopController = Intake.getClosedLoopController();

    IntakeEn = Intake.getEncoder();

    SparkFlexConfig IntakeConfig = new SparkFlexConfig();

    IntakeConfig.inverted(true).idleMode(IdleMode.kCoast);
    IntakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    Intake.configure(IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeSpeed(double targetRPM) {
    IntakeClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    SmartDashboard.putNumber("Intake Target RPM", targetRPM);
  }

  public void stopIntake() {
    Intake.set(0);

    SmartDashboard.putNumber("Intake Target RPM", 0);
  }

  public boolean optic() {
    return intakeSwitchState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    double IntakeRPM = IntakeEn.getVelocity();
    switchIO.updateInputs(switchInputs);
    Logger.processInputs("OpticSensor", switchInputs);

    if (switchInputs.intakeSwitchState) {
      intakeSwitchState = true;
    } else {
      intakeSwitchState = false;
    }

    SmartDashboard.putNumber("Intake Current RPM", IntakeRPM);
  }
}
