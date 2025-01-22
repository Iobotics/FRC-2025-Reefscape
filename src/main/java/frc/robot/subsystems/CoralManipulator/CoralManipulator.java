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
import frc.robot.Constants;

public class CoralManipulator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax TopleftOutake;

  private SparkMax ToprightOutake;
  private static SparkClosedLoopController TopleftClosedLoopController;
  private SparkClosedLoopController ToprightClosedLoopController;
  private RelativeEncoder TopleftOutakeEncoder;
  private RelativeEncoder ToprightOutakeEncoder;

  private SparkMax BottomleftOutake;
  private SparkMax BottomrightOutake;
  private static SparkClosedLoopController BottomleftClosedLoopController;
  private SparkClosedLoopController BottomrightClosedLoopController;
  private RelativeEncoder BottomleftOutakeEncoder;
  private RelativeEncoder BottomrightOutakeEncoder;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxOutput = 1.0;
  private static final double kMinOutput = -1.0;
  /*
    public final CoralManipulatorIO io;
    public final

  */
  public CoralManipulator(CoralManipulatorIO coralManipulatorIOSpark) {
    TopleftOutake = new SparkMax(Constants.topLeftOutakeID, MotorType.kBrushless);
    ToprightOutake = new SparkMax(Constants.topRightOutakeID, MotorType.kBrushless);
    TopleftClosedLoopController = TopleftOutake.getClosedLoopController();
    ToprightClosedLoopController = ToprightOutake.getClosedLoopController();
    TopleftOutakeEncoder = TopleftOutake.getEncoder();
    ToprightOutakeEncoder = ToprightOutake.getEncoder();

    BottomleftOutake = new SparkMax(1, MotorType.kBrushless);
    BottomrightOutake = new SparkMax(2, MotorType.kBrushless);
    BottomleftClosedLoopController = BottomleftOutake.getClosedLoopController();
    BottomrightClosedLoopController = BottomrightOutake.getClosedLoopController();
    BottomleftOutakeEncoder = BottomleftOutake.getEncoder();
    BottomrightOutakeEncoder = BottomrightOutake.getEncoder();

    // top left config
    SparkMaxConfig TopleftConfig = new SparkMaxConfig();

    TopleftConfig.inverted(true).idleMode(IdleMode.kCoast);
    TopleftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    TopleftOutake.configure(
        TopleftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // top right config
    SparkMaxConfig ToprightConfig = new SparkMaxConfig();

    ToprightConfig.inverted(false).idleMode(IdleMode.kCoast);
    ToprightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    ToprightOutake.configure(
        ToprightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // bottom left config
    SparkMaxConfig BottomleftConfig = new SparkMaxConfig();

    BottomleftConfig.inverted(true).idleMode(IdleMode.kCoast);
    BottomleftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    BottomleftOutake.configure(
        BottomleftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // bottom right config
    SparkMaxConfig BottomrightConfig = new SparkMaxConfig();

    BottomrightConfig.inverted(true).idleMode(IdleMode.kCoast);
    BottomrightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    BottomrightOutake.configure(
        BottomrightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public CoralManipulator(CoralManipulatorIOSim coralManipulatorIOSpark) {
    // TODO Auto-generated constructor stub
  }

  public void setTopOutakeSpeed(double targetRPM) {
    ToprightClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    TopleftClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    SmartDashboard.putNumber("Manipulator Target RPM", targetRPM);
  }

  public void setBottomOutakeSpeed(double targetRPM) {
    BottomrightClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    BottomleftClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    SmartDashboard.putNumber("Manipulator Target RPM", targetRPM);
  }

  public void stopOutake() {
    TopleftOutake.set(0);
    ToprightOutake.set(0);
    BottomleftOutake.set(0);
    BottomrightOutake.set(0);
    SmartDashboard.putNumber("Manipulator Target RPM", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    double TopleftRPM = TopleftOutakeEncoder.getVelocity();
    SmartDashboard.putNumber("top left Manipulator Current RPM", TopleftRPM);

    // Display secondary shooter RPM
    double ToprightRPM = ToprightOutakeEncoder.getVelocity();
    SmartDashboard.putNumber("top right Manipulator RPM", ToprightRPM);

    double BottomleftRPM = BottomleftOutakeEncoder.getVelocity();
    SmartDashboard.putNumber("bottom left Manipulator Current RPM", BottomleftRPM);

    // Display secondary shooter RPM
    double BottomrightRPM = BottomrightOutakeEncoder.getVelocity();
    SmartDashboard.putNumber("bottom right Manipulator RPM", BottomrightRPM);
  }
}
