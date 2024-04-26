// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.ManualIntake;
import frc.robot.commands.IntakeCommands.OutakeCommand;
import frc.robot.commands.ShooterCommands.AmpShootCommand;
import frc.robot.commands.ShooterCommands.CookShooter;
import frc.robot.commands.ShooterCommands.FeederOverStage;
import frc.robot.commands.ShooterCommands.FeederShooter;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.RobotCentricDrive;
import frc.robot.commands.AligningCommands.VelocityOffset;
import frc.robot.commands.ArmCommands.ArmToPose;
import frc.robot.commands.ClimberCommands.ClimberOpenLoop;
import frc.robot.commands.ClimberCommands.ExtendClimber;

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
  private final LEDSubsystem m_led = new LEDSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem(m_drive);

  SwerveRequest.FieldCentricFacingAngle m_head = new SwerveRequest.FieldCentricFacingAngle()
  .withDriveRequestType(DriveRequestType.Velocity);


  private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 

    m_head.ForwardReference = ForwardReference.RedAlliance;
    m_head.HeadingController.setPID(8, 0, 0);
    m_head.HeadingController.enableContinuousInput(-Math.PI, Math.PI);


  /** The commands for the autonomous period. */

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

  /**   Joystick 1 controls. */
    m_drive.setDefaultCommand(new FieldCentricDrive(
      m_drive,
      () -> -chassisDriver.getLeftY(),
      () -> -chassisDriver.getLeftX(),
      () -> -chassisDriver.getRightX()));

    chassisDriver.b().whileTrue(new RobotCentricDrive(
      m_drive, 
      () -> -chassisDriver.getLeftY(),
      () -> -chassisDriver.getLeftX(),
      () -> -chassisDriver.getRightX()));
   
    chassisDriver.a().onTrue(m_drive.runOnce(() -> m_drive.seedFieldRelative()));
     
      chassisDriver.x().whileTrue(
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
    .alongWith(new IntakeCommand(m_intake, m_shooter, m_led)))
    .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));

    //Auto Align
    chassisDriver.leftBumper().whileTrue(new AutoPickup(m_drive));

  /**  Joystick 2    */
    subsystemsDriver.a()
      .onTrue(m_arm.goToPosition(99.0));
    subsystemsDriver.b()
      .whileTrue(new OutakeCommand(m_intake, m_shooter, m_led));

    subsystemsDriver.povUp()
      .onTrue(new ExtendClimber(m_climber));
    subsystemsDriver.povDown()
      .whileTrue(new ClimberOpenLoop(m_climber));

    subsystemsDriver.leftBumper()
      .whileTrue(new AmpShootCommand(m_shooter, m_intake, m_arm, m_led))
      .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));

    subsystemsDriver.x()
      .whileTrue(new ArmToPose(m_arm)
      .alongWith(new CookShooter(m_shooter, m_led)))
      .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));

    subsystemsDriver.y()
      .whileTrue(m_arm.goToPosition(113.0));

    subsystemsDriver.povLeft()
      .whileTrue(new FeederShooter(m_shooter, m_led))
      .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_UNDER_STAGE));

    subsystemsDriver.rightBumper()
      .whileTrue(new ManualIntake(m_intake,m_shooter));

    subsystemsDriver.povRight()
      .whileTrue(new FeederOverStage(m_shooter, m_led)
      .alongWith(m_arm.goToPosition(160)))
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

    return null;

    }
  

  public CommandSwerveDrivetrain getDrive(){
    return m_drive;
  }

  public IntakeSubsystem getIntake(){
    return m_intake;
  }

  public LEDSubsystem getLED(){
    return m_led;
  }

  public ArmSubsystem getArm(){
    return m_arm;
  }

  public VisionSubsystem getVision(){
    return m_vision;
  }
    
}
