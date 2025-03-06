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
    private final Corl corlSubsystem;
    PIDController controller1;
    PIDController controller2;
    PIDController controller3;

    double desiredRotatorPosition;
    double desiredIntakePosition;
    double desiredElevatorPosition;

    public GoToSetpoint(Corl corlSubsystem, double desiredElevatorPosition, double desiredRotatorPosition, double desiredIntakePosition){
        controller1.setPID(0.4, 0, 0);
        controller2.setPID(0.4, 0, 0);
        controller3.setPID(0.4, 0, 0);

        this.desiredIntakePosition = desiredIntakePosition;
        this.desiredRotatorPosition = desiredRotatorPosition;
        this.desiredElevatorPosition = desiredElevatorPosition;

        this.corlSubsystem = corlSubsystem;
        addRequirements(this.corlSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double corlRotatorOutput = MathUtil.clamp(controller1.calculate(corlSubsystem.getRotatorPosition(), desiredRotatorPosition), -0.8, 0.8);
        double corlIntakeOutput = MathUtil.clamp(controller2.calculate(corlSubsystem.getIntakePosition(), desiredIntakePosition), -0.5, 0.5);
        double elevatorOutput = MathUtil.clamp(controller2.calculate(corlSubsystem.getElevatorPosition(), desiredElevatorPosition), -0.5, 0.5);

        corlSubsystem.runRotator(corlRotatorOutput);
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