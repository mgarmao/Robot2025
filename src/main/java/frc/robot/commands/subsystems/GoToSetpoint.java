package frc.robot.commands.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Corl;



/**
 * Goes to a specific setpoint
 * 
 */
public class GoToSetpoint extends Command{
    private final Corl corlSubsystem; // defines the subsystem through the type coral 
    PIDController controller1 = new PIDController(0.15, 0.04, 0.0);
    PIDController controller2 =  new PIDController(0.4, 0.02, 0);
    PIDController controller3 =  new PIDController(0.4, 0, 0);

    double desiredRotatorPosition;
    double desiredIntakePosition;
    double desiredElevatorPosition;

    public GoToSetpoint(Corl corlSubsystem, double desiredElevatorPosition, double desiredRotatorPosition, double desiredIntakePosition){


        this.desiredIntakePosition = desiredIntakePosition;
        this.desiredRotatorPosition = desiredRotatorPosition;
        this.desiredElevatorPosition = desiredElevatorPosition;

        this.corlSubsystem = corlSubsystem;
        addRequirements(this.corlSubsystem); // lets you use this stuff as a subsystem 
    }

    @Override
    public void initialize(){ 

    }

    @Override
    public void execute(){
        double corlRotatorOutput = MathUtil.clamp(controller1.calculate(corlSubsystem.getRotatorPosition(), desiredRotatorPosition), -0.8, 0.8);
        double corlIntakeOutput = MathUtil.clamp(controller2.calculate(corlSubsystem.getIntakePosition(), desiredIntakePosition), -0.6, 0.6);
        double elevatorOutput = MathUtil.clamp(controller3.calculate(corlSubsystem.getElevatorPosition(), desiredElevatorPosition), -0.4, 0.8);
        // We defined PID controller as controller 1 

        corlSubsystem.intakeRotatorNoCommand(corlIntakeOutput);
        corlSubsystem.elevatorRunNoCommand(elevatorOutput);

        SmartDashboard.putNumber("EL POS", corlSubsystem.getElevatorPosition());

        if(corlSubsystem.getElevatorPosition()<40)
        {
            corlSubsystem.runRotatorNoCommand(corlRotatorOutput);
        }
        else{
            corlSubsystem.runRotatorNoCommand(0);
        }

        // If the rotator is -25 or greater and the elevator is not below a certain height

        // if(corlSubsystem.getElevatorPosition() >=65 && corlRotatorOutput>0){
        //     corlSubsystem.runRotatorNoCommand(0);
        // }
        // else{
        // }

        // if(corlSubsystem.getElevatorPosition()>-40 || (corlSubsystem.getRotatorPosition()>=-50&&corlRotatorOutput>0)||(corlSubsystem.getRotatorPosition()<=0&&corlRotatorOutput<0)){
        // }
        // corlSubsystem.runRotatorNoCommand(elevatorOutput);

        // corlSubsystem.intakeRotate(corlIntakeOutput);
        // corlSubsystem.runElevator(elevatorOutput);
    }

    @Override
    public boolean isFinished(){
        return controller1.atSetpoint()&&controller2.atSetpoint();
    }
   
    @Override
    public void end(boolean interrupted){
        corlSubsystem.armStop();
    }
}

