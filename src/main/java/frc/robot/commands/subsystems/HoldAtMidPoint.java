

/*
The goal of this command (HoldAtMidpoint) is to use a PID controller to hold the elevator of the robot at a specific midpoint position
(in this case, 50.0).
The PID controller calculates an output based on the error (difference) between the current position and the target midpoint,
and the output is used to control the elevator motor through the Corl subsystem.
*/
package frc.robot.commands.subsystems;




import edu.wpi.first.math.MathUtil; // For clamping the PID output
import edu.wpi.first.math.controller.PIDController; // For PID control in the first place
import frc.robot.subsystems.Corl; // Importing the corl.java subsystem which contains the method to control the motors
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Red Squiggles vvvvvvvvv (Off to a bad start) (Check your capitlization!!!) (Then fix the caps in RobotContainer.java)
public class holdAtMidpoint extends Command {
   private final Corl corlSubsystem;
   private final PIDController elevatorHoldController; //** 




   // Define your target midpoint value
   private final double midpointElevatorPosition = 50.0; // Our specified midpoint (adjust as needed)


   // Function to hold the elevator at the midpoint position
   public holdAtMidpoint(Corl corlSubsystem) {
       this.corlSubsystem = corlSubsystem;
       //Ok tequnically correct but just put this up where you first define the PID controller //**
       this.elevatorHoldController = new PIDController(0.4, 0.02, 0.0);


       // PLEASE if you dont need it and its commented out, just delete it 

    //    double kp = 0.4;  Proportional: How far off am I right now? Big error =  big correction
    //    double ki = 0.02; Integral: How long have I been off Adds up errors to fi small gaps over time
    //    double kd = 0.0;  Derivative term is not used in this case, because we can't calculate how fast the error is changing without a physical robot




       addRequirements(corlSubsystem); // Declares that this command uses the Corl subsystem
   }




   @Override
   public void initialize() {
       // Optional: Reset the PID controller
       elevatorHoldController.reset();
   }



   //Your guys indenting is just all over the place *******!!!!!
   @Override
   public void execute() {
        double currentPosition = corlSubsystem.getElevatorPosition(); //good




        double output = MathUtil.clamp(
            elevatorHoldController.calculate(currentPosition, midpointElevatorPosition),
           -0.4, 0.4
        );
       //elevatorHoldController.calculate(...) → might give you a value like 0.7 or -1.2


       //MathUtil.clamp(..., -0.4, 0.4) → forces the result to stay between -0.4 and 0.4


       //So if the PID says "give it 0.7 power", clamp brings it down to 0.4


      //If PID says "-1.0", clamp lifts it up to -0.4

    
        // Nice ^ (Better if indented properly)





       corlSubsystem.elevatorRunNoCommand(output);
   }

   @Override
   public boolean isFinished() {
       // Keep running until interrupted
       return false;
   }

   @Override
   public void end(boolean interrupted) {
       corlSubsystem.armStop(); // THis is a command, won't work here
       corlSubsystem.elevatorRunNoCommand(0); // Stop the motor
   }
}




