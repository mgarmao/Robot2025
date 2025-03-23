// package frc.robot.commands.subsystems;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;


// /**
//  * Goes to a specific setpoint
//  * angelo - holy error
//  */
// public class AlignWithApriltag extends Command
// {
//     private final SwerveSubsystem drivetrain;
//     private Pose3d desiredPose3d;

//     PIDController xPidController;
//     PIDController yPidController;
//     PIDController thetaPidController;

//     double desiredRotatorPosition;
//     double desiredIntakePosition;
    

//     public AlignWithApriltag(SwerveSubsystem drivetrain, int targetID)
//     {
//         xPidController.setPID(0.4, 0, 0);
//         yPidController.setPID(0.4, 0, 0);
//         thetaPidController.setPID(0.4,0,0);



//         this.drivetrain = drivetrain;
//         this.desiredPose3d = desiredPose3d;
//         this.desiredRotatorPosition = desiredRotatorPosition;

//         addRequirements(this.drivetrain);
//     }

// /**
// * The initial subroutine of a command.  Called once when the command is initially scheduled.
// */
//     @Override
//     public void initialize()
//     {

//     }

//     /**
//      * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
//     * until {@link #isFinished()}) returns true.)
//     */
//     @Override
//     public void execute()
//     {
//         desiredPose3d = drivetrain.getDesiredReefPose2d(17);

//         xPidController.setSetpoint(desiredPose3d.getX());
//         yPidController.setSetpoint(desiredPose3d.getY());
//         thetaPidController.setSetpoint(desiredPose3d.getRotation().getAngle());

//         double xOutput = MathUtil.clamp(xPidController.calculate(drivetrain.getPose().getX()),-0.5,0.5);
//         double yOutput = MathUtil.clamp(yPidController.calculate(drivetrain.getPose().getY()),-0.5,0.5);
//         double thetaOutput = MathUtil.clamp(thetaPidController.calculate(drivetrain.getPose().getRotation().getRadians()),-0.5,0.5);

//         drivetrain.drive(new Translation2d(xOutput, yOutput), thetaOutput, true); 
//     }

//     /**
//      * <p>
//     * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
//     * the scheduler will call its {@link #end(boolean)} method.
//     * </p><p>
//     * Returning false will result in the command never ending automatically. It may still be cancelled manually or
//     * interrupted by another command. Hard coding this command to always return true will result in the command executing
//     * once and finishing immediately. It is recommended to use *
//     * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
//     * </p>
//     *
//     * @return whether this command has finished.
//     */
//     @Override
//     public boolean isFinished()
//     {
//         return xPidController.atSetpoint() && yPidController.atSetpoint();
//     }

//     /**
//      * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
//     * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
//     * up loose ends, like shutting off a motor that was being used in the command.
//     *
//     * @param interrupted whether the command was interrupted/canceled
//     */
//     @Override
//     public void end(boolean interrupted)
//     {
//         drivetrain.drive(new Translation2d(0.0, 0.0), 0.0, true); 
//     }
// }
