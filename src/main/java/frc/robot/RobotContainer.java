// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OutakeCommand;
import frc.robot.commands.ShooterCommands.AmpShootCommand;
import frc.robot.commands.ShooterCommands.CookShooter;
import frc.robot.commands.ShooterCommands.ShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.RobotCentricDrive;
import frc.robot.commands.AligningCommands.VelocityOffset;
import frc.robot.commands.ArmCommands.ArmToPose;
import frc.robot.commands.ClimberCommands.ClimberOpenLoop;
import frc.robot.commands.ClimberCommands.ExtendClimber;
import frc.robot.commands.ClimberCommands.RetractClimber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Create Joystick Objects for TeleOp control
  private final CommandXboxController chassisDriver = new CommandXboxController(0);
  private final CommandXboxController subsystemsDriver = new CommandXboxController(1);

  //Create Subsystem Objects
  private final CommandSwerveDrivetrain m_drive = TunerConstants.DriveTrain;
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter =  new ShooterSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem(m_drive);

  SwerveRequest.FieldCentricFacingAngle m_head = new SwerveRequest.FieldCentricFacingAngle()
  .withDriveRequestType(DriveRequestType.Velocity);

  private Timer rumbleTimer = new Timer();

  private final SendableChooser<String> autoChooser;
  private final String m_DefaultAuto = "NO AUTO";
  private String m_autoSelected;
  private final String[] m_autoNames = {"NO AUTO", "4 NOTE INTERPOLATED", "4 NOTE STEAL",
   "3 NOTE COMPLEMENT", "4 NOTE SUBWOOFER", "2 NOTE COMPLEMENT", "2 NOTE CENTER", 
   "3 NOTE CENTER", "4 NOTE CENTER","SAFE COMPLEMENT", "5 NOTE AMP", "6 NOTE AMP"}; 

  private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(DriveConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withVelocityX(-chassisDriver.getLeftY())
      .withVelocityY(-chassisDriver.getLeftX())
      .withRotationalRate(-chassisDriver.getRightX());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 

    m_head.ForwardReference = ForwardReference.RedAlliance;
    m_head.HeadingController.setPID(8, 0, 0);
    m_head.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    //Idle Arm
    NamedCommands.registerCommand("ArmIdle", m_arm.goToPosition(170));
    //Cook Shooter
    NamedCommands.registerCommand("CookShooter", new CookShooter(m_shooter));
    NamedCommands.registerCommand("PrepareArm", new ArmToPose(m_arm));
    //Shoot
    NamedCommands.registerCommand("Shoot", 
    new ParallelDeadlineGroup(
      new ShooterCommand(m_shooter, m_intake, m_arm), 
      new ArmToPose(m_arm)));
    NamedCommands.registerCommand("SubwooferShoot", 
    new ParallelDeadlineGroup(
      new ShooterCommand(m_shooter, m_intake,m_arm), 
       m_arm.goToPosition(160.0)));    //Intake
    NamedCommands.registerCommand("Intake", 
    new ParallelCommandGroup(
      new IntakeCommand(m_intake, m_shooter), //new AutoOutake(m_intake), 
      m_arm.goToPosition(IntakeConstants.INTAKE_ANGLE)));
    //Aim<
    NamedCommands.registerCommand("AutoAim", 
      new ParallelRaceGroup(new AutoAim(m_drive, m_vision), new WaitCommand(1)));
    //Change Pipelines
    NamedCommands.registerCommand("MainTracking", 
    new InstantCommand(
      () -> m_vision.setCameraPipeline(VisionConstants.main_Pipeline)));
    //Change Pipeline
    NamedCommands.registerCommand("FarTracking", 
    new InstantCommand(
      () -> m_vision.setCameraPipeline(VisionConstants.far_Pipeline)));

    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("No Auto", m_DefaultAuto);
    autoChooser.addOption("Interpolated 4 Notes", m_autoNames[1]);
    //autoChooser.addOption("Subwoofer 4 Notes", m_autoNames[4]);
    //autoChooser.addOption("Steal and 3 Notes", m_autoNames[2]);
    //autoChooser.addOption("Complement 2 Notes", m_autoNames[5]);
    //autoChooser.addOption("Complement 3 Notes", m_autoNames[3]);
    autoChooser.addOption("2 Note Center", m_autoNames[6]);
    autoChooser.addOption("3 Note Center", m_autoNames[7]);
    autoChooser.addOption("4 Note Center", m_autoNames[8]);
    autoChooser.addOption("Safe Complement", m_autoNames[9]);
    autoChooser.addOption("5 Note AMP", m_autoNames[10]);
    autoChooser.addOption("6 Note AMP", m_autoNames[11]);

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

   private double addForAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return 180;
        }
        return 0;
    }

    private double invertForAlliance() {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
          return -1;
      }
      return 1;
  }

  private void configureBindings() {

    m_drive.setDefaultCommand(new FieldCentricDrive(
      m_drive,
      () -> chassisDriver.getLeftY(),
      () -> chassisDriver.getLeftX(),
      () -> -chassisDriver.getRightX()));

    //chassisDriver.x().whileTrue(
      //m_drive.setControl(robotCentricDrive));
        

    chassisDriver.x().whileTrue(new RobotCentricDrive(
      m_drive, 
      () -> -chassisDriver.getLeftY(),
      () -> -chassisDriver.getLeftX(),
      () -> -chassisDriver.getRightX()));

    //Joystick 1
    chassisDriver.a().onTrue(m_drive.runOnce(() -> m_drive.seedFieldRelative()));
    
    chassisDriver.b().whileTrue(
      m_drive.applyRequest(
              () -> m_head.withVelocityX((-chassisDriver.getLeftY() * DriveConstants.MaxSpeed) * 0.5)
                  .withVelocityY((-chassisDriver.getLeftX() * DriveConstants.MaxSpeed) * 0.5)
                  .withTargetDirection(m_drive.getVelocityOffset())                        
                  .withDeadband(DriveConstants.MaxSpeed * 0.1)
                  .withRotationalDeadband(0))
                  .alongWith(new VelocityOffset(m_drive, () -> chassisDriver.getRightTriggerAxis())));

    //Manual Pickup
    chassisDriver.rightBumper()
    .whileTrue(m_arm.goToPosition(IntakeConstants.INTAKE_ANGLE)
    .alongWith(new IntakeCommand(m_intake, m_shooter)))
    .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));

    chassisDriver.leftBumper().whileTrue(new AutoPickup(m_drive));

    //Auto Pickup
    /*chassisDriver.leftBumper().onTrue(m_arm.goToPosition(178)
    .alongWith(new IntakeCommand(m_intake, m_shooter))
    .alongWith(new AutoPickup(m_drive, -chassisDriver.getLeftY() * DriveConstants.MaxSpeed,
                                    -chassisDriver.getLeftX() * DriveConstants.MaxSpeed)))
    .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));*/

    // Joystick 2
    subsystemsDriver.a().onTrue(m_arm.goToPosition(93));
    
    subsystemsDriver.b().whileTrue(new OutakeCommand(m_intake, m_shooter));

    //Climber controls
    subsystemsDriver.povUp().onTrue(new ExtendClimber(m_climber));
    subsystemsDriver.povDown().whileTrue(new ClimberOpenLoop(m_climber));
    //Open loop climber

    subsystemsDriver.leftBumper()
      .whileTrue(new AmpShootCommand(m_shooter, m_intake, m_arm))
      .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));

    subsystemsDriver.x().whileTrue(new CookShooter(m_shooter));

    subsystemsDriver.rightBumper()
      .whileTrue(new ArmToPose(m_arm)
      .alongWith(new ShooterCommand(m_shooter, m_intake, m_arm)))
      .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));

    //TODO: DriveTrain Characterization, comment if not used
    /*chassisDriver.start().and(chassisDriver.y()).whileTrue(m_drive.sysIdQuasistatic(Direction.kForward));
    chassisDriver.start().and(chassisDriver.x()).whileTrue(m_drive.sysIdQuasistatic(Direction.kReverse));
    chassisDriver.back().and(chassisDriver.y()).whileTrue(m_drive.sysIdDynamic(Direction.kForward));
    chassisDriver.back().and(chassisDriver.x()).whileTrue(m_drive.sysIdDynamic(Direction.kReverse));
    
    chassisDriver.start().and(chassisDriver.a()).onTrue(new RunCommand(SignalLogger::stop));*/

    m_drive.registerTelemetry(logger::telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

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

      /*case "4 NOTE STEAL":
        autonomousCommand = new PathPlannerAuto("Steal4Score");
      break;*/

      /*case "3 NOTE COMPLEMENT":
        autonomousCommand = new PathPlannerAuto("NoteComplement");
      break;*/

      case "2 NOTE COMPLEMENT":
        autonomousCommand = new PathPlannerAuto("SafeComplement");
      break;

      /*case "4 NOTE SUBWOOFER":
        autonomousCommand = new PathPlannerAuto("SubwooferAuto");
      break;*/

      case "2 NOTE CENTER":
        autonomousCommand = new PathPlannerAuto("2NoteAuto");
      break;

      case "3 NOTE CENTER":
        autonomousCommand = new PathPlannerAuto("3NoteAuto");
      break;
      
      case "4 NOTE CENTER":
        autonomousCommand = new PathPlannerAuto("4NoteAuto");
      break;

      case "SAFE COMPLEMENT":
        autonomousCommand = new PathPlannerAuto("SafeComplement");
      break;

      case "5 NOTE AMP":
        autonomousCommand = new PathPlannerAuto("5NoteAmp");
      break;

      case "6 NOTE AMP":
        autonomousCommand = new PathPlannerAuto("6NoteAmp");
      break;
    }

    return autonomousCommand;
  }

  public CommandSwerveDrivetrain getDrive(){
    return m_drive;
  }

  public IntakeSubsystem getIntake(){
    return m_intake;
  }

  public VisionSubsystem getVision(){
    return m_vision;
  }

  public Timer getRumbleTimer(){
    return rumbleTimer;
  }

  /*public Optional<Rotation2d> overrideForSpeakerAim(){
    if(m_shooter.isShooting()){
      return Optional.of(m)
    }
  }*/
}
