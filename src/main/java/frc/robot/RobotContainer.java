// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OutakeCommand;
import frc.robot.commands.ShooterCommands.ShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.POVSelector;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.TeleopControl;
import frc.robot.commands.ArmCommands.ArmToPose;
import frc.robot.commands.AutoCommands.AutoOutake;
import frc.robot.commands.AutoCommands.AutoShooter;
import frc.robot.commands.ClimberCommands.ClimberCommand;
import frc.robot.commands.ClimberCommands.DescendCommand;

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
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter =  new ShooterSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final POVSelector m_selector = new POVSelector(subsystemsDriver);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
   m_drive.setDefaultCommand(new TeleopControl
    (m_drive, 
    () -> chassisDriver.getRawAxis(1), 
    () -> chassisDriver.getRawAxis(0), 
    () -> chassisDriver.getRawAxis(4), 
    () -> true));

    //Idle Arm
    NamedCommands.registerCommand("ArmIdle", m_arm.goToPosition(170));
    //Shoot
    NamedCommands.registerCommand("AutoShoot", 
    new ParallelDeadlineGroup(
      new AutoShooter(m_shooter, m_intake,m_arm), //.raceWith(new WaitCommand(2))
      new ArmToPose(m_arm, m_selector)));

    NamedCommands.registerCommand("SubwooferShoot", 
    new ParallelDeadlineGroup(
      new AutoShooter(m_shooter, m_intake,m_arm), //.raceWith(new WaitCommand(2.5))
      new ArmToPose(m_arm, m_selector)));    //Intake
    NamedCommands.registerCommand("Intake", 
    new ParallelCommandGroup(
      new IntakeCommand(m_intake,m_shooter), new AutoOutake(m_intake), 
      m_arm.goToPosition(179)));
    //Aim
    NamedCommands.registerCommand("AutoAim", 
      new ParallelRaceGroup(new AutoAim(m_drive), new WaitCommand(1)));
    //Change Pipelines
    NamedCommands.registerCommand("MainTracking", 
    new InstantCommand(
      () -> m_drive.getVisionSubsystem().setCameraPipeline(VisionConstants.main_Pipeline)));
    //Change Pipeline
    NamedCommands.registerCommand("FarTracking", 
    new InstantCommand(
      () -> m_drive.getVisionSubsystem().setCameraPipeline(VisionConstants.far_Pipeline)));
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
    //Joystick 1
      new JoystickButton(chassisDriver, 1).onTrue(
      new InstantCommand(() -> m_drive.zeroHeading()));
      new JoystickButton(chassisDriver, 2).whileTrue(new AutoAim(m_drive));

      new JoystickButton(chassisDriver, 6)
      .whileTrue(m_arm.goToPosition(178.0)
      .alongWith(new IntakeCommand(m_intake,m_shooter)))
      .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));     

    // Joystick 2

   //Pov upper
   new POVButton(subsystemsDriver, 0).onTrue(new InstantCommand(() -> m_selector.updateSelectionUp()));

   //Pov down
   new POVButton(subsystemsDriver, 180).onTrue(new InstantCommand(() -> m_selector.updateSelectionDown()));


      new JoystickButton(subsystemsDriver, 1).whileTrue(m_arm.goToPosition(93)); 
      new JoystickButton(subsystemsDriver, 2).whileTrue(new OutakeCommand(m_intake, m_shooter));
      new JoystickButton(subsystemsDriver, 3).whileTrue(new DescendCommand(m_climber));
      new JoystickButton(subsystemsDriver, 4).whileTrue((new ClimberCommand(m_climber)));
       new JoystickButton(subsystemsDriver, 6)
      .whileTrue(new ArmToPose(m_arm, m_selector)
      .alongWith(new ShooterCommand(m_shooter, m_intake,m_arm,m_selector)))
      .whileFalse(m_arm.goToPosition(160));
   //   new JoystickButton(subsystemsDriver, 6).whileTrue(m_arm.goToPosition(135).alongWith(new ShooterCommand(m_shooter, m_intake, m_arm, m_selector)));
            //new JoystickButton(subsystemsDriver, 1).whileTrue(new DriveTuner(m_drive));        
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Interpolated Auto");//m_autoChooser.getSelected();
  }


  public DriveTrain getDrive(){
    return m_drive;
  }
}
