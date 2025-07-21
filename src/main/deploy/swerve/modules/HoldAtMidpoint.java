/*
Ok little guys, so we split you into 2 teams, team ved/shaurya and team rudra/arjun.
Team ved shaurya needs to make a new subsystem (like the corl subsystem that’s already there) with 2 spark max motors. 
Team arjun and rudra need to use the already existing corl subsystem but make a new file command (like the GoToSetpoint.java) using corl functions.
*/ 

import edu.wpi.first.math.MathUtil; // Obvious, because 0.5 is the midpoint of 0 and 1
import edu.wpi.first.math.controller.PIDController; // We need this to set the robot at its midpoint  
import frc.robot.subsystems.CorlSubsystem; // Importing the CorlSubsystem which contains the methods to control the motors

public class HoldAtMidPoint extends Command {
    private final CorlSubsystem corl; 
    PIDController controller4 = new PIDController(0.1, 0.01, 0.001); 
    
    double Kp = 0.1; // PROPORTIONAL: Makes the robot respond to error right away 
    double Ki = 0.001; // INTEGRAL: Fixes small steady errors over time 
    double Kd = 0.01; // DERIVATIVE: Helps slow things down before overshooting

    
    public HoldAtMidPoint(CorlSubsystem corl) {
        this.corl = corl;
        addRequirements(corl); // lets you use this stuff as a subsystem 
    }

    @Override
    public void initialize() {
        // Set the desired positions for the motors to hold at midpoint
        corl.setTargetPosition(0.5); // Assuming midpoints is 0.5 meaning (range is 0 to 1) 
    }
    
    @Override    
    public void execute () {
        corl.moveToTargetPosition(); // This method is called repeatedly to hold the motors at the midpoint
    }   
}