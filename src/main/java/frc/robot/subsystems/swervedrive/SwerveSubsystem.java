// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

import java.io.Console;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
// import swervelib.math.YAGSLConversions; // Ensure this class exists in the correct package or remove if not needed
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Geometry;
import org.json.simple.parser.ParseException;
import org.opencv.core.Mat;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{

  private PIDController pidController1;
  private PIDController pidController2;
  private PIDController pidController3;
  
    /**
     * Swerve drive object.
     */
    public final SwerveDrive         swerveDrive;
    /**
     * AprilTag field layout.
     */
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    /**
     * Enable vision odometry updates while driving. micheal said racoon a lot
     */
    private final boolean             visionDriveTest     = true;////////////////////////////////////////////////////////////////////////////////////////
    /**
     * PhotonVision class to keep an accurate odometry.
     */
    private       Vision              vision;
  
    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory)
    {
      // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
      //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
      //  The encoder resolution per motor revolution is 1 per motor revolution.
      double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
      // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
      //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
      //  The gear ratio is 6.75 motor revolutions per wheel rotation.
      //  The encoder resolution per motor revolution is 1 per motor revolution.
      double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
      System.out.println("\"conversionFactors\": {");
      System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
      System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
      System.out.println("}");
  
      // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      try
      {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                    new Pose2d(new Translation2d(Meter.of(1),
                                                                                                 Meter.of(4)),
                                                                               Rotation2d.fromDegrees(0)));
        // Alternative method if you don't want to supply the conversion factor via JSON files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
      } catch (Exception e)
      {
        throw new RuntimeException(e);
      }
      swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
      swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
      swerveDrive.setAngularVelocityCompensation(true,
                                                 true,
                                                 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
      swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                  1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
      swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
      if (visionDriveTest)
      {
        try{
          setupPhotonVision();
          // Stop the odometry thread if we are using vision that way we can synchronize updates better.
          swerveDrive.stopOdometryThread();
        }
        catch(Exception e)
        {
          DriverStation.reportError(e.toString(), true);
          System.out.println("Vision setup failed");
        }
      }
      setupPathPlanner();
    }
  
    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
    {
      swerveDrive = new SwerveDrive(driveCfg,
                                    controllerCfg,
                                    Constants.MAX_SPEED,
                                    new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                               Rotation2d.fromDegrees(0)));
    }
  
    /**
     * Setup the photon vision class.
     */
    public void setupPhotonVision()
    {
      SmartDashboard.putBoolean("photonIsActive", false);
      vision = new Vision(swerveDrive::getPose, swerveDrive.field);
    }

    public void updateVisionOdom(){
      try{
        vision.updatePoseEstimation(swerveDrive);
      }catch (Exception e)
      {
        DriverStation.reportError(e.toString(), true);
        System.out.println("Vision update failed");
      }
    }
  
    @Override
    public void periodic()
    {
      // When vision is enabled we must manually ufpdate odometry in SwerveDrive
     
      swerveDrive.updateOdometry();
      
    }
  
    @Override
    public void simulationPeriodic()
    {
    }
  
    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner()
    {
      // Load the RobotConfig from the GUI settings. You should probably
      // store this in your Constants file
      RobotConfig config;
      try
      {
        config = RobotConfig.fromGUISettings();
  
        final boolean enableFeedforward = true;
        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose,
            // Robot pose supplier
            this::resetOdometry,
            // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity,
            // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speedsRobotRelative, moduleFeedForwards) -> {
              if (enableFeedforward)
              {
                swerveDrive.drive(
                    speedsRobotRelative,
                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                    moduleFeedForwards.linearForces()
                                 );
              } else
              {
                swerveDrive.setChassisSpeeds(speedsRobotRelative);
              }
            },
            // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController(
                // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(5.0, 0.0, 0.0),
                // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0)
                // Rotation PID constants
            ),
            config,
            // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent())
              {
                return alliance.get() == DriverStation.Alliance.Red; //// FLAG AS POSSIBLE ERROR
              }
              return false;
            },
            this
            // Reference to this subsystem to set requirements
                             );
  
      } catch (Exception e)
      {
        // Handle exception as needed
        e.printStackTrace();
      }
  
      //Preload PathPlanner Path finding
      // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
      PathfindingCommand.warmupCommand().schedule();
    }
  
    /**
     * Get the distance to the speaker.
     *
     * @return Distance to speaker in meters.
     */
    public double getDistanceToSpeaker()
    {
      int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4; 
      // Taken from PhotonUtils.getDistanceToPose
      Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
      return getPose().getTranslation().getDistance(speakerAprilTagPose.toPose2d().getTranslation());
    }
  
    /**
     * Get the yaw to aim at the speaker.
     *
     * @return {@link Rotation2d} of which you need to achieve.
     */
    public Rotation2d getTargetYaw(int id)
    {
      int allianceAprilTag = id;
      // Taken from PhotonUtils.getYawToPose()
      Pose3d        speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
      Translation2d relativeTrl         = speakerAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
      return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getOdometryHeading());
    }
  
    /**
     * Aim the robot at the speaker.
     *
     * @param tolerance Tolerance in degrees.
     * @return Command to turn the robot to the speaker.
     */
    public Command aimAtTarget3d(int id, double tolerance)
    {
      SwerveController controller = swerveDrive.getSwerveController();
      return run(
          () -> {
            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
                                                     controller.headingCalculate(getHeading().getRadians(),
                                                                                 getTargetYaw(id).getRadians()),
                                                                         getHeading());
            drive(speeds);
          }).until(() -> Math.abs(getTargetYaw(id).minus(getHeading()).getDegrees()) < tolerance);
    }
  
    /**
     * Aim the robot at the target returned by PhotonVision.
     *
     * @return A {@link Command} which will run the alignment.
     */
    // public Command aimAtTarget(Cameras camera)
    // {
  
    //   return run(() -> {
    //     Optional<PhotonPipelineResult> resultO = camera.getBestResult();
    //     if (resultO.isPresent())
    //     {
    //       var result = resultO.get();
    //       if (result.hasTargets())
    //       {
    //         drive(getTargetSpeeds(0,
    //                               0,
    //                               Rotation2d.fromDegrees(result.getBestTarget()
    //                                                            .getYaw()))); // Not sure if this will work, more math may be required.
    //       }
    //     }
    //   });
    // }
  
    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName)
    {
      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return new PathPlannerAuto(pathName);
    }
  
    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose)
    {
  // Create the constraints to use while pathfinding
      PathConstraints constraints = new PathConstraints(
          swerveDrive.getMaximumChassisVelocity(), 4.0,
          swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
  
  // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(
          pose,
          constraints,
          edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                       );
    }
  
    /**
     * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
     *
     * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
     * @return {@link Command} to run.
     * @throws IOException    If the PathPlanner GUI settings is invalid
     * @throws ParseException If PathPlanner GUI settings is nonexistent.
     */
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
    throws IOException, ParseException
    {
      SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                              swerveDrive.getMaximumChassisAngularVelocity());
      AtomicReference<SwerveSetpoint> prevSetpoint
          = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                                                     swerveDrive.getStates(),
                                                     DriveFeedforwards.zeros(swerveDrive.getModules().length)));
      AtomicReference<Double> previousTime = new AtomicReference<>();
  
      return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                      () -> {
                        double newTime = Timer.getFPGATimestamp();
                        SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                                                                                        robotRelativeChassisSpeed.get(),
                                                                                        newTime - previousTime.get());
                        swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                                          newSetpoint.moduleStates(),
                                          newSetpoint.feedforwards().linearForces());
                        prevSetpoint.set(newSetpoint);
                        previousTime.set(newTime);
  
                      });
    }
  
    /**
     * Drive with 254's Setpoint generator; port written by PathPlanner.
     *
     * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
     * @return Command to drive the robot using the setpoint generator.
     */
    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds)
    {
      try
      {
        return driveWithSetpointGenerator(() -> {
          return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
  
        });
      } catch (Exception e)
      {
        DriverStation.reportError(e.toString(), true);
      }
      return Commands.none();
  
    }
  
  
    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand()
    {
      return SwerveDriveTest.generateSysIdCommand(
          SwerveDriveTest.setDriveSysIdRoutine(
              new Config(),
              this, swerveDrive, 12, true),
          3.0, 5.0, 3.0);
    }
  
  
    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand()
    {
      return SwerveDriveTest.generateSysIdCommand(
          SwerveDriveTest.setAngleSysIdRoutine(
              new Config(),
              this, swerveDrive),
          3.0, 5.0, 3.0);
    }
  
    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    public Command centerModulesCommand()
    {
      return run(() -> Arrays.asList(swerveDrive.getModules())
                             .forEach(it -> it.setAngle(0.0)));
    }
  
    /**
     * Returns a Command that drives the swerve drive to a specific distance at a given speed.
     *
     * @param distanceInMeters       the distance to drive in meters
     * @param speedInMetersPerSecond the speed at which to drive in meters per second
     * @return a Command that drives the swerve drive to a specific distance at a given speed
     */
    public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
    {
      return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
          .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                       distanceInMeters);
    }
  
    /**
     * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
     *
     * @param kS the static gain of the feedforward
     * @param kV the velocity gain of the feedforward
     * @param kA the acceleration gain of the feedforward
     */
    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
    {
      swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }
  
    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
      return run(() -> {
        // Make the robot move
        swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                              translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                              translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      });
    }
  
    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                                DoubleSupplier headingY)
    {
      // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
      return run(() -> {
  
        Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                   translationY.getAsDouble()), 0.8);
  
        // Make the robot move
        driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                        headingX.getAsDouble(),
                                                                        headingY.getAsDouble(),
                                                                        swerveDrive.getOdometryHeading().getRadians(),
                                                                        swerveDrive.getMaximumChassisVelocity()));
      });
    }
  
    /**
     * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
      swerveDrive.drive(translation,rotation,fieldRelative, false); // Open loop is disabled since it shouldn't be used most of the time.
      
    }
  
    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity)
    {
      swerveDrive.driveFieldOriented(velocity);
    }
  
    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
    {
      return run(() -> {
        swerveDrive.driveFieldOriented(velocity.get());
      });
    }
  
    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity)
    {
      swerveDrive.drive(velocity);
    }
  
  
    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics()
    {
      return swerveDrive.kinematics;
    }
  
    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
      swerveDrive.resetOdometry(initialHolonomicPose);
    }
  
    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose()
    {
      return swerveDrive.getPose();
    }
  
    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
      swerveDrive.setChassisSpeeds(chassisSpeeds);
    }
  
    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory)
    {
      swerveDrive.postTrajectory(trajectory);
    }
  
    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro()
    {
      swerveDrive.zeroGyro();
    }
  
    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is available.
     */

    // FLAG AS POSSIBLE ERROR
    public boolean isRedAlliance()
    {
      var alliance = DriverStation.getAlliance();
      return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }
  
    /**
     * This will zero (calibrate) the robot to assume the current position is facing forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    
    // FLAG AS POSSIBLE ERROR
    public void zeroGyroWithAlliance()
    {
      if (isRedAlliance())
      {
        zeroGyro();
        //Set the pose 180 degrees
        resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
      } else
      {
        zeroGyro();
        
        resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
      }
    }
  
    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake)
    {
      swerveDrive.setMotorIdleMode(brake);
    }
  
    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading()
    {
      return getPose().getRotation();
      // if(isRedAlliance()) {
      //   return Rotation2d.fromDegrees((getPose().getRotation().getDegrees() + 180) % 360);
      // }
      // else {
      //   return getPose().getRotation();
      // }
    }
  
    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
      Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
      return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                          scaledInputs.getY(),
                                                          headingX,
                                                          headingY,
                                                          getHeading().getRadians(),
                                                          Constants.MAX_SPEED);
    }
  
    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
    {
      Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
  
      return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                          scaledInputs.getY(),
                                                          angle.getRadians(),
                                                          getHeading().getRadians(),
                                                          Constants.MAX_SPEED);
    }
  
    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity()
    {
      return swerveDrive.getFieldVelocity();
    }
  
    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity()
    {
      return swerveDrive.getRobotVelocity();
    }
  
    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController()
    {
      return swerveDrive.swerveController;
    }
  
    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration()
    {
      return swerveDrive.swerveDriveConfiguration;
    }
  
    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock()
    {
      swerveDrive.lockPose();
    }
  
    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch()
    {
      return swerveDrive.getPitch();
    }
  
    /**
     * Add a fake vision reading for testing purposes.
     */
    public void addFakeVisionReading()
    {
      swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }
  
  //   public Command DriveToProcessor() {
  //     pidController1 = new PIDController(1.0, 0.0, 0.0);
  //     int id = 16;
  //     return run(
  //       () -> {
  //         var result = Vision.Cameras.FRONT.camera.getLatestResult();         
  //         boolean hasTargets = result.hasTargets();    
  //         if(hasTargets){
  //           List<PhotonTrackedTarget> targets = result.getTargets();
  //           double yDistance = 0;     
  //           for(PhotonTrackedTarget target:targets){
  //               Transform3d thisTarget = target.getBestCameraToTarget();
  //               SmartDashboard.putNumber("This ID", target.getFiducialId());
  //               if(target.getFiducialId()==id){
  //                 yDistance = thisTarget.getY();    
  //                 double translationVal = MathUtil.clamp(pidController1.calculate(yDistance, 0.0), -0.5,0.5);
  //                 drive(new Translation2d(0.5, translationVal), 0.0, true);
  //               }
  //           }
  //         }          
  //   }).until(() -> pidController1.atSetpoint()||!Vision.Cameras.FRONT.camera.getLatestResult().hasTargets());
  // }    

  // public Command alignWithTargetFront(int id) {
  //   return run(
  //     () -> {
  //       var result = Vision.Cameras.FRONT.camera.getLatestResult();         
  //       boolean hasTargets = result.hasTargets();    
  //       if(hasTargets){
  //         List<PhotonTrackedTarget> targets = result.getTargets();
  //         for(PhotonTrackedTarget target:targets){
  //           Transform3d cameraToTarget = target.getBestCameraToTarget();
  //           SmartDashboard.putNumber("This ID", target.getFiducialId());
  //           if(target.getFiducialId()==id){
  //             alignWithTarget(cameraToTarget);
  //           }
  //         }
  //       }
  //       else{
  //         drive(new Translation2d(0.4, 0.0), 0.0, true); //xyzxyzxyz
  //       }          
  //   });
  // }    

  // public Command alignWithTargetBack(int id) {
  //   return run(
  //     () -> {
  //       var result = Vision.Cameras.BACK.camera.getLatestResult();         
  //       boolean hasTargets = result.hasTargets();    
  //       if(hasTargets){
  //         List<PhotonTrackedTarget> targets = result.getTargets();
  //         for(PhotonTrackedTarget target:targets){
  //           Transform3d cameraToTarget = target.getBestCameraToTarget();
  //           SmartDashboard.putNumber("This ID", target.getFiducialId());
  //           if(target.getFiducialId()==id){
  //             alignWithTarget(cameraToTarget);
  //           }
  //         }
  //       }
  //       else{
  //         drive(new Translation2d(0.4, 0.0), 0.0, true);
  //       }          
  //   });
  // }  

  public Command alignMode(DoubleSupplier driverX, DoubleSupplier driverY, DoubleSupplier rotate)
  {
    return run(
      ()->{

        double clamp = 0.8;
        double drivx = driverX.getAsDouble();
        double drivy = driverY.getAsDouble();
        double driveR = rotate.getAsDouble();        

        double neospeedx  = drivx;
        double neospeedy  = drivy;

        neospeedx = MathUtil.clamp(neospeedx, -clamp, clamp);
        neospeedy = MathUtil.clamp(neospeedy, -clamp, clamp);
        // driveR    = MathUtil.clamp(driveR, -0.5, 0.5); in actuallity we dont need to limit the rotation

        drive(
          new Translation2d(neospeedy, neospeedx), 
          -driveR, 
          false
          );
          
      });
  }

  public Command reverseAlignMode(DoubleSupplier driverX, DoubleSupplier driverY, DoubleSupplier rotate)
  {
    return run(
      ()->{

        double clamp = 0.8;
        double drivx = driverX.getAsDouble();
        double drivy = driverY.getAsDouble();
        double driveR = rotate.getAsDouble();        

        double neospeedx  = drivx;
        double neospeedy  = drivy;

        neospeedx = MathUtil.clamp(neospeedx, -clamp, clamp);
        neospeedy = MathUtil.clamp(neospeedy, -clamp, clamp);
        // driveR    = MathUtil.clamp(driveR, -0.5, 0.5); in actuallity we dont need to limit the rotation

        drive(
          new Translation2d(-neospeedy, -neospeedx), 
          -driveR, 
          false
          );
          
      });
  }
   

  public void alignWithTarget(Transform3d camToTarget) {
    pidController1 = new PIDController(1.0, 0.0, 0.0);
    pidController2 = new PIDController(0.2, 0.0, 0.0);
    pidController3 = new PIDController(0.2, 0.0, 0.0);
    
    Pose2d currentPose2d = swerveDrive.getPose(); 
    Pose3d currentPose = new Pose3d(currentPose2d.getX(),currentPose2d.getY(),0,new edu.wpi.first.math.geometry.Rotation3d(0, 0, currentPose2d.getRotation().getRadians()));

    Transform3d tagTransform3d = camToTarget.inverse();
    Pose3d tagPose3d = new Pose3d(tagTransform3d.getTranslation(), tagTransform3d.getRotation());

    // Convert the 3D pose to a 2D pose by ignoring the z component.
    Pose2d tagPose2d = new Pose2d(
      tagPose3d.getTranslation().toTranslation2d(),
      tagPose3d.getRotation().toRotation2d()
    );

    // Create a Transform2d offset of 1 meter in the tag’s forward direction (its x-axis).
    Transform2d offset = new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d());
    Pose2d targetPose = tagPose2d.transformBy(offset);
    Pose2d errorPose = targetPose.relativeTo(currentPose2d);

    // The errorPose now represents:
    // - errorPose.getTranslation().getX(): How far ahead (or behind, if negative) the target is in the robot's coordinate frame
    // - errorPose.getTranslation().getY(): How far to the left/right the target is
    // - errorPose.getRotation(): The difference in orientation needed to face the target
    double yError = -errorPose.getTranslation().getX();
    double xError = -errorPose.getTranslation().getY();
    double thetaError = -errorPose.getRotation().getDegrees();

    // Compute PID outputs.
    double xOutput = pidController1.calculate(yError,0);
    double yOutput = pidController2.calculate(xError, 5);
    double thetaOutput = pidController3.calculate(thetaError, 0);
    

    SmartDashboard.putNumber("y Error", yError);
    SmartDashboard.putNumber("X Error", xError);
    SmartDashboard.putNumber("Theta Error", thetaError);

    // Command the drivetrain using the calculated outputs.
    swerveDrive.drive(new ChassisSpeeds(xOutput, yOutput, thetaOutput));
  }

  public Pose3d getDesiredReefPose2d(int bestID){
    double additionalX = 5;
    double additionalY = 5;
    Pose2d reefPose;
    Pose3d desiredPose;
    double newX = 0, newY = 0;
    
    Optional<Pose3d> aprilTagePose = aprilTagFieldLayout.getTagPose(bestID);

    //closestToHuman
    if(bestID == 18 || bestID==7){
      // subtract some y 
      newX = aprilTagePose.get().getX();
      newY = aprilTagePose.get().getX()-additionalY;
    }

    if (bestID == 10 || bestID==21){
      // add some y
      newX = aprilTagePose.get().getX();
      newY = aprilTagePose.get().getX()+additionalY;
    }

    if(bestID == 17 || bestID==8){
      // addLittle x and subtract alittle y
      newX = aprilTagePose.get().getX()+additionalX;
      newY = aprilTagePose.get().getX()-additionalY;
    }

    if(bestID == 19 || bestID==6){
      // subract Alittle x and subtract alittle y
      newX = aprilTagePose.get().getX()-additionalX;
      newY = aprilTagePose.get().getX()-additionalY;
    }

    if(bestID == 11 || bestID==20){
      // subract Alittle x and subtract add alittle y
      newX = aprilTagePose.get().getX()-additionalX;
      newY = aprilTagePose.get().getX()-additionalY;
    }

    if(bestID == 9 || bestID==22){
      // add Alittle x and add alittle y
      newX = aprilTagePose.get().getX()+additionalX;
      newY = aprilTagePose.get().getX()+additionalY;
    }

    

    desiredPose = new Pose3d(newX, newY, 0, aprilTagePose.get().getRotation());

    return desiredPose;
  }

  public int bestTargetID(){
    int id = 0;
    var result = Vision.Cameras.REAR.camera.getLatestResult();         
    boolean hasTargets = result.hasTargets();    
    if(hasTargets){
      List<PhotonTrackedTarget> targets = result.getTargets();
      for(PhotonTrackedTarget target:targets){
        SmartDashboard.putNumber("This ID", target.getFiducialId());
      }
    }
    return id;    
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }
}
