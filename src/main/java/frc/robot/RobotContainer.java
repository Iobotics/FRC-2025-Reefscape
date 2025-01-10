// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.CoralOutake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
// The robot's subsystems and commands are defineddXboxController(OperatorConstants.kDriverControllerPort);
  // Configure the trigger bindings here...
  private final Joystick joystick1 = new Joystick(0); 
  private final JoystickButton outakeSpinButton = new JoystickButton(joystick1, 1);
  private final CoralOutake CoralOutake = new CoralOutake();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
/** The container for the robot. Contains subsystems, OI devices, and commands. */
public RobotContainer() {
  configureBindings();
}

/**
 * Use this method to define your trigger->command mappings. Triggers can be created via the
 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
 * predicate, or via the named factories in {@link
 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
 * joysticks}.
 */



     
       
 
 private void configureBindings() {
  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


  outakeSpinButton.onTrue(
      new InstantCommand(() -> CoralOutake.setOutakeSpeed(0.2))
  );
  outakeSpinButton.onFalse(
          new InstantCommand(() -> CoralOutake.stopOutake())
  );
}


/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */
// public Command getAutonomousCommand() {
//   // An example command will be run in autonomous
//   return Autos.exampleAuto(m_exampleSubsystem);
// }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
