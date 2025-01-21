package frc.robot.subsystems.CoralIntake;

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

public class CoralIntake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax leftIntake;

  private SparkMax rightIntake;
  private static SparkClosedLoopController leftClosedLoopController;
  private SparkClosedLoopController rightClosedLoopController;
  private RelativeEncoder leftIntakeEncoder;
  private RelativeEncoder rightIntakeEncoder;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxOutput = 1.0;
  private static final double kMinOutput = -1.0;

  public CoralIntake() {
    leftIntake = new SparkMax(1, MotorType.kBrushless);
    rightIntake = new SparkMax(2, MotorType.kBrushless);
    leftClosedLoopController = leftIntake.getClosedLoopController();
    rightClosedLoopController = rightIntake.getClosedLoopController();
    leftIntakeEncoder = leftIntake.getEncoder();
    rightIntakeEncoder = rightIntake.getEncoder();

    SparkMaxConfig leftConfig = new SparkMaxConfig();

    leftConfig.inverted(true).idleMode(IdleMode.kCoast);
    leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    leftIntake.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();

    rightConfig.inverted(false).idleMode(IdleMode.kCoast);
    rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    rightIntake.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeSpeed(double targetRPM) {
    rightClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    leftClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
  }

  public void stopIntake() {
    leftIntake.set(0);
    rightIntake.set(0);
    SmartDashboard.putNumber("Shooter Target RPM", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    double leftRPM = leftIntakeEncoder.getVelocity();
    SmartDashboard.putNumber("Main Shooter Current RPM", leftRPM);

    // Display secondary shooter RPM
    double rightRPM = rightIntakeEncoder.getVelocity();
    SmartDashboard.putNumber("Secondary Shooter RPM", rightRPM);
  }
}
