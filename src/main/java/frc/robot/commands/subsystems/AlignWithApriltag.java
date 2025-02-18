// package frc.robot.commands.subsystems;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;


// /**
//  * Goes to a specific setpoint
//  * 
//  */
// public class AlignWithApriltag extends Command
// {
//     private final SwerveSubsystem drivetrain;
//     PIDController controller1;
//     PIDController controller2;

//     double desiredRotatorPosition;
//     double desiredIntakePosition;

//     public AlignWithApriltag(SwerveSubsystem drivetrain, int targetID)
//     {
//         controller1.setPID(0.4, 0, 0);
//         controller2.setPID(0.4, 0, 0);

//         this.drivetrain = drivetrain;
//         this.desiredRotatorPosition = desiredRotatorPosition;

//         this.corlSubsystem = corlSubsystem;
//         addRequirements(this.corlSubsystem);
//     }

//   /**
//    * The initial subroutine of a command.  Called once when the command is initially scheduled.
//    */
//     @Override
//     public void initialize()
//     {

//     }

//     /**
//      * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
//      * until {@link #isFinished()}) returns true.)
//      */
//     @Override
//     public void execute()
//     {
//         drivetrain.alignWithTargetBack();
//     }

//     /**
//      * <p>
//      * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
//      * the scheduler will call its {@link #end(boolean)} method.
//      * </p><p>
//      * Returning false will result in the command never ending automatically. It may still be cancelled manually or
//      * interrupted by another command. Hard coding this command to always return true will result in the command executing
//      * once and finishing immediately. It is recommended to use *
//      * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
//      * </p>
//      *
//      * @return whether this command has finished.
//      */
//     @Override
//     public boolean isFinished()
//     {
//         return controller1.atSetpoint()&&controller2.atSetpoint();
//     }

//     /**
//      * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
//      * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
//      * up loose ends, like shutting off a motor that was being used in the command.
//      *
//      * @param interrupted whether the command was interrupted/canceled
//      */
//     @Override
//     public void end(boolean interrupted)
//     {
//         corlSubsystem.armStop();
//     }
// }
