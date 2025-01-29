package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralFeeder.CoralFeeder;
import frc.robot.subsystems.CoralFeeder.CoralFeederSwitchIObot;

public class CoralFeederCommands extends Command {
  /** Creates a new Intaking2. */
  CoralFeeder intake;
  boolean enabled;
  boolean direction;

  public CoralFeederCommands(CoralFeeder intake, boolean enabled, boolean direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.enabled = enabled;
    this.direction = direction;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("done", false);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   intake.setIntakeSpeed(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("done", true);
    intake.stopIntake();
  }

  // Returns true when the command should end.
 /*  @Override
  public boolean isFinished() {
  } */
}