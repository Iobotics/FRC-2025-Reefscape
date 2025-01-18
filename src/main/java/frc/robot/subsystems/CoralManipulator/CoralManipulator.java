package frc.robot.subsystems.CoralManipulator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralManipulator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax TopleftOutake;

  private static SparkClosedLoopController TopleftClosedLoopController;
  private RelativeEncoder TopleftOutakeEncoder;

  private SparkMax BottomleftOutake;
  private static SparkClosedLoopController BottomleftClosedLoopController;
  private RelativeEncoder BottomleftOutakeEncoder;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxOutput = 1.0;
  private static final double kMinOutput = -1.0;

  public CoralManipulator(CoralManipulatorIO coralManipulatorIOSpark) {
    TopleftOutake = new SparkMax(1, MotorType.kBrushless);
    TopleftClosedLoopController = TopleftOutake.getClosedLoopController();
    TopleftOutakeEncoder = TopleftOutake.getEncoder();

    BottomleftOutake = new SparkMax(1, MotorType.kBrushless);
    BottomleftClosedLoopController = BottomleftOutake.getClosedLoopController();
    BottomleftOutakeEncoder = BottomleftOutake.getEncoder();

    // top left config
    SparkMaxConfig TopleftConfig = new SparkMaxConfig();

    TopleftConfig.inverted(false).idleMode(IdleMode.kCoast);
    TopleftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    TopleftOutake.configure(
        TopleftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // bottom left config
    SparkMaxConfig BottomleftConfig = new SparkMaxConfig();

    BottomleftConfig.inverted(true).idleMode(IdleMode.kCoast);
    BottomleftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    BottomleftOutake.configure(
        BottomleftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public CoralManipulator(CoralManipulatorIOSim coralManipulatorIOSpark) {
    // TODO Auto-generated constructor stub
  }

  public void setTopOutakeSpeed(double targetRPM) {
    TopleftClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    SmartDashboard.putNumber("Manipulator Target RPM", targetRPM);
  }

  public void setBottomOutakeSpeed(double targetRPM) {
    BottomleftClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    SmartDashboard.putNumber("Manipulator Target RPM", targetRPM);
  }

  public void stopOutake() {
    TopleftOutake.set(0);
    BottomleftOutake.set(0);
    SmartDashboard.putNumber("Manipulator Target RPM", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    double TopleftRPM = TopleftOutakeEncoder.getVelocity();
    SmartDashboard.putNumber("top left Manipulator Current RPM", TopleftRPM);

    double BottomleftRPM = BottomleftOutakeEncoder.getVelocity();
    SmartDashboard.putNumber("bottom left Manipulator Current RPM", BottomleftRPM);
  }
}
