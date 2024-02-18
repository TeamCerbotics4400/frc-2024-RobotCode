// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommands.IntakeSequence;
import frc.robot.commands.IntakeCommands.OutakeCommand;
import frc.robot.commands.IntakeCommands.CenterSequence;
import frc.robot.commands.ShooterCommands.ShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.TeleopControl;
import frc.robot.commands.ArmCommands.ArmToPose;

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
  //private final VisionSubsystem m_vision = new VisionSubsystem(m_drive);
  //private final ClimberSubsystem m_climber = new ClimberSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
   m_drive.setDefaultCommand(new TeleopControl
    (m_drive, 
    () -> chassisDriver.getRawAxis(1), 
    () -> chassisDriver.getRawAxis(0), 
    () -> chassisDriver.getRawAxis(4), 
    () -> true));

    //Idle Arm
    NamedCommands.registerCommand("ArmIdle", m_arm.goToPosition(160));
    //Shoot
    NamedCommands.registerCommand("Shoot", 
    new ParallelDeadlineGroup(
      new ShooterCommand(m_shooter, m_intake,m_arm).raceWith(new WaitCommand(1.0)), 
      new ArmToPose(m_arm, 
      () -> m_arm.getAngleForDistance(
        LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.tagLimelightName).getZ()))));

    NamedCommands.registerCommand("SubwooferShoot", 
    new ParallelDeadlineGroup(
      new ShooterCommand(m_shooter, m_intake,m_arm).raceWith(new WaitCommand(1.0)), 
      m_arm.goToPosition(160.0)));
    //Intake
    NamedCommands.registerCommand("Intake", 
    new ParallelDeadlineGroup(
      new SequentialCommandGroup(new IntakeSequence(m_intake, m_shooter), 
      new CenterSequence(m_intake, m_shooter)), 
      m_arm.goToPosition(180)));
    //Aim
    NamedCommands.registerCommand("AutoAim", 
      new AutoAim(m_drive));
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
      new JoystickButton(chassisDriver, 1).onTrue(
      new InstantCommand(() -> m_drive.zeroHeading()));

      new JoystickButton(chassisDriver, 5)
      .whileTrue(m_arm.goToPosition(180.0)
      .alongWith(new SequentialCommandGroup(
        new IntakeSequence(m_intake, m_shooter), new CenterSequence(m_intake, m_shooter))))
      .whileFalse(m_arm.goToPosition(160));
      
      new JoystickButton(chassisDriver, 6)
      .whileTrue(new ArmToPose(m_arm, 
      () -> m_arm.getAngleForDistance(
        LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.tagLimelightName).getZ()))
      .alongWith(new ShooterCommand(m_shooter, m_intake,m_arm)))
      .whileFalse(m_arm.goToPosition(160));

      new JoystickButton(chassisDriver, 2).whileTrue(new AutoAim(m_drive));
      new JoystickButton(chassisDriver, 4).whileTrue(new OutakeCommand(m_intake));
      //new JoystickButton(chassisDriver, 3).whileTrue(

      //new JoystickButton(subsystemsDriver, 1).whileTrue(new DriveTuner(m_drive));
      //Romans ver of shooting routine new JoystickButton(subsystemsDriver, 2).whileTrue(new ShooterCommand(m_shooter, m_intake).alongWith(new OutakeCommand(m_intake)));
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
