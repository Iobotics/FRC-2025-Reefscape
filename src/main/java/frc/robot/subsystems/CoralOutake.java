package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;


public class CoralOutake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax leftOutake;
  private SparkMax rightOutake;
  private static SparkClosedLoopController leftClosedLoopController;
  private SparkClosedLoopController rightClosedLoopController;
  private RelativeEncoder leftOutakeEncoder; 
  private RelativeEncoder rightOutakeEncoder;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxOutput = 1.0;
  private static final double kMinOutput = -1.0;

  public CoralOutake() {
    leftOutake = new SparkMax(1,MotorType.kBrushless);
    rightOutake = new SparkMax(2,MotorType.kBrushless);
    leftClosedLoopController = leftOutake.getClosedLoopController();
    rightClosedLoopController = rightOutake.getClosedLoopController();
    leftOutakeEncoder = leftOutake.getEncoder();
    rightOutakeEncoder = rightOutake.getEncoder();

    SparkMaxConfig leftConfig = new SparkMaxConfig();

    leftConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast);
    leftConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kP, kI, kD);

    leftOutake.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();

    rightConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast);
    rightConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kP, kI, kD);

    rightOutake.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  public void setOutakeSpeed(double targetRPM){
    rightClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    leftClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
  }
  public void stopOutake(){
     leftOutake.set(0);
     rightOutake.set(0);
     SmartDashboard.putNumber("Shooter Target RPM", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update current RPM from the motor's encoder (example)
    double leftRPM = leftOutakeEncoder.getVelocity();
    SmartDashboard.putNumber("Main Shooter Current RPM", leftRPM);

    // Display secondary shooter RPM
    double rightRPM = rightOutakeEncoder.getVelocity();
    SmartDashboard.putNumber("Secondary Shooter RPM", rightRPM);
  }
}