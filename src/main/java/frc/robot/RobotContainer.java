// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OutakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.TeleopControl;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  
  private final Joystick chassisDriver = new Joystick(0);
  private final Joystick subsystemsDriver = new Joystick(1);
  private final DriveTrain m_drive = new DriveTrain();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter =  new ShooterSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    m_drive.setDefaultCommand(new TeleopControl
    (m_drive, 
    () -> chassisDriver.getRawAxis(1), 
    () -> chassisDriver.getRawAxis(0), 
    () -> -chassisDriver.getRawAxis(4), 
    () -> !chassisDriver.getRawButton(4)));

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

      new JoystickButton(chassisDriver, 1).onTrue(
      new InstantCommand(() -> m_drive.zeroHeading()));

   new JoystickButton(subsystemsDriver, 1).whileTrue(new ShooterCommand(shooter, m_arm));
   new JoystickButton(subsystemsDriver, 4).whileTrue(new IntakeCommand(intake));
   new JoystickButton(subsystemsDriver, 1).whileTrue(new OutakeCommand(intake));

   new JoystickButton(subsystemsDriver, 5)
   .onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Intake test");//m_autoChooser.getSelected();
  }

}
