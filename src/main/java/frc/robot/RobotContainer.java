// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.subsystems.GoToSetpoint;
import frc.robot.commands.subsystems.goToPosition;
// import frc.robot.commands.subsystems.GoToSetpoint;
import frc.robot.commands.subsystems.rotateBackToHardstop;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Corl;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

import java.io.File;

import org.opencv.features2d.FastFeatureDetector;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController oppXbox = new CommandXboxController(1);



  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem     drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  public static final Corl CORL = new Corl();
  public static final Algae ALGAE = new Algae();

  private final Command algaeAutomaticIn = new rotateBackToHardstop(ALGAE);
  


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverXbox.getLeftY() ,
                                                                () -> -driverXbox.getLeftX())
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverXbox.getRightX() * -1,
                                                                                            () -> driverXbox.getRightY()*-1)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  // private final Command GoToHumanPlayerSetpoint = new GoToSetpoint(CORL, Constants.Setpoints.HumanRotator, Constants.Setpoints.HumanIntake);   
  // private final Command GoToHighCorlSetpoint = new GoToSetpoint(CORL, Constants.Setpoints.HighRotator, Constants.Setpoints.HighIntake );   

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    

    
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("IntakeIn",CORL.runIntake(-0.5));
    NamedCommands.registerCommand("IntakeOut",CORL.runIntake(0.5));
    NamedCommands.registerCommand("ArmVertical", new GoToSetpoint(CORL, 0, Constants.Setpoints.ArmVeritcal, 0));

    // NamedCommands.registerCommand("SetpointHigh", new GoToSetpoint(CORL, 0, 0, 0));
    // NamedCommands.registerCommand("SetpointDown", new GoToSetpoint(CORL, 0, 0, 0));
    // NamedCommands.registerCommand("SetpointMid", new GoToSetpoint(CORL, 0, 0, 0));
    // NamedCommands.registerCommand("SetpointHumanPlayer", new GoToSetpoint(CORL, 0, 0, 0));



    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings(){
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedDirectAngle :
                                driveFieldOrientedDirectAngleSim);

    if (Robot.isSimulation())
    {
      driverXbox.start()
        .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.b()
        .whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.x()
        .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y()
        .whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start()
        .onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back()
        .whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper()
        .onTrue(Commands.none());
      driverXbox.rightBumper()
        .onTrue(Commands.none());
      
    } 
    else{
      //////////////////////////////////////////////////////////
      driverXbox.a()
        .onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.b()
        .whileTrue(drivebase.alignWithTargetFront(17));
      driverXbox.y()
        .whileTrue(drivebase.alignWithTargetBack(16));

      // oppXbox.rightBumper().whileTrue(algaeAutomaticIn).onFalse(ALGAE.moveRotator(0));

      oppXbox.a().whileTrue(ALGAE.smartRunIntake(0.9)).onFalse(ALGAE.smartRunIntake(0.03));
      oppXbox.b().whileTrue(ALGAE.smartRunIntake(-0.9)).onFalse(ALGAE.smartRunIntake(0.03));
      oppXbox.y().whileTrue(new goToPosition(-20, ALGAE)).whileFalse(algaeAutomaticIn);
      oppXbox.x().whileTrue(new GoToSetpoint(CORL,0,-43.7,-6.0)).onFalse(CORL.armStop());


      oppXbox.povLeft()
        .whileTrue(CORL.armUp())
        .onFalse(CORL.armStop());
        oppXbox.povRight()
        .whileTrue(CORL.armDown())
        .onFalse(CORL.armStop());

      oppXbox.povUp().whileTrue(CORL.runElevator(0.8)).onFalse(CORL.runElevator(0.0));
      oppXbox.povDown().whileTrue(CORL.runElevator(-0.35)).onFalse(CORL.runElevator(0.0));

      oppXbox.leftBumper().whileTrue(CORL.intakeRotate(0.3)).onFalse(CORL.intakeRotate(0));
      oppXbox.rightBumper().whileTrue(CORL.intakeRotate(-0.3)).onFalse(CORL.intakeRotate(0));

      oppXbox.leftTrigger().whileTrue(CORL.runIntake(0.8)).onFalse(CORL.runIntake(0));
      oppXbox.rightTrigger().whileTrue(CORL.runIntake(-0.8)).onFalse(CORL.runIntake(0));

      driverXbox.leftTrigger().whileTrue(drivebase.alignMode(true, ()->driverXbox.getLeftX(), ()->driverXbox.getLeftY(), 1.5));
      
      

      // oppXbox.rightTrigger().whileTrue(GoToHighCorlSetpoint);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
