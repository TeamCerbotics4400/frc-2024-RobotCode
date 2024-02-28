// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.commands.ShooterCommands.AmpShootCommand;
import frc.robot.commands.ShooterCommands.ShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.POISelector;
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
  private final POISelector m_selector = new POISelector(subsystemsDriver);

  private Timer rumbleTimer = new Timer();

  private final SendableChooser<String> autoChooser;
  private final String m_DefaultAuto = "NO AUTO";
  private String m_autoSelected;
  private final String[] m_autoNames = {"NO AUTO", "4 NOTE INTERPOLATED", "4 NOTE STEAL",
   "3 NOTE COMPLEMENT", "4 NOTE SUBWOOFER"};

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Idle Arm
    NamedCommands.registerCommand("ArmIdle", m_arm.goToPosition(170));
    //Shoot
    NamedCommands.registerCommand("Shoot", 
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
      m_arm.goToPosition(179.5)));
    //Aim<
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

    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("No Auto", m_DefaultAuto);
    autoChooser.addOption("Interpolated 4 Notes", m_autoNames[1]);
    autoChooser.addOption("Subwoofer 4 Notes", m_autoNames[4]);
    autoChooser.addOption("Steal and 3 Notes", m_autoNames[2]);
    autoChooser.addOption("Complement 3 Notes", m_autoNames[3]);

    SmartDashboard.putData("Auto Chooser", autoChooser);

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

    m_drive.setDefaultCommand(new TeleopControl
    (m_drive, 
    () -> chassisDriver.getRawAxis(1), 
    () -> chassisDriver.getRawAxis(0), 
    () -> chassisDriver.getRawAxis(4), 
    () -> true));

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
   //new POVButton(subsystemsDriver, 0).onTrue(new InstantCommand(() -> m_selector.updateSelectionUp()));

   //Pov down
  // new POVButton(subsystemsDriver, 180).onTrue(new InstantCommand(() -> m_selector.updateSelectionDown()));


    new JoystickButton(subsystemsDriver, 1).whileTrue(m_arm.goToPosition(93)); 
    new JoystickButton(subsystemsDriver, 2).whileTrue(new OutakeCommand(m_intake, m_shooter));
    new JoystickButton(subsystemsDriver, 3).whileTrue(new DescendCommand(m_climber));
    new JoystickButton(subsystemsDriver, 4).whileTrue((new ClimberCommand(m_climber)));
        new JoystickButton(subsystemsDriver, 5)
        .whileTrue(new ArmToPose(m_arm, m_selector)
        .alongWith(new ShooterCommand(m_shooter, m_intake,m_arm,m_selector)))
        .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));   
    new JoystickButton(subsystemsDriver, 6)
        .whileTrue(new ArmToPose(m_arm, m_selector)
        .alongWith(new AmpShootCommand(m_shooter, m_intake,m_arm,m_selector)))
        .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));   
     // new JoystickButton(subsystemsDriver, 6).whileTrue(m_arm.goToPosition(134).alongWith(new ShooterCommand(m_shooter, m_intake, m_arm, m_selector)));
            //new JoystickButton(subsystemsDriver, 1).whileTrue(new DriveTuner(m_drive));        
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command autonomousCommand = null;
    m_autoSelected = autoChooser.getSelected();
    
    System.out.println("Auto Selected" + m_autoSelected);
    switch (m_autoSelected) {
      case "NO AUTO":
        autonomousCommand = null;
      break;

      case "4 NOTE INTERPOLATED":
        autonomousCommand = new PathPlannerAuto("InterpolatedAuto");
      break;

      case "4 NOTE STEAL":
        autonomousCommand = new PathPlannerAuto("Steal4Score");
      break;

      case "3 NOTE COMPLEMENT":
        autonomousCommand = new PathPlannerAuto("NoteComplement");
      break;

      case "4 NOTE SUBWOOFER":
        autonomousCommand = new PathPlannerAuto("SubwooferAuto");
      break;
    }

    return autonomousCommand;
  }

  public DriveTrain getDrive(){
    return m_drive;
  }

  public IntakeSubsystem getIntake(){
    return m_intake;
  }

  public Timer getRumbleTimer(){
    return rumbleTimer;
  }
}
