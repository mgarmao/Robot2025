package frc.robot.commands.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewIntake;

/**
 * Goes to a specific setpoint
 * 
 */
public class GoToSetpoint extends Command {
    private final NewIntake newIntakeSubsystem; // defines the subsystem through the type
    PIDController controller1 = new PIDController(0.15, 0.04, 0.0);
    // PIDController controller2 = new PIDController(0.4, 0.02, 0);
    // PIDController controller3 = new PIDController(0.4, 0, 0);

    // double desiredRotatorPosition;
    // double desiredIntakePosition;
    // double desiredElevatorPosition;
    double desiredNewIntakePosition;

    public GoToSetpoint(NewIntake newIntakeSubsystem, double desiredNewIntakePosition) {

        // this.desiredIntakePosition = desiredIntakePosition;
        // this.desiredRotatorPosition = desiredRotatorPosition;
        // this.desiredElevatorPosition = desiredElevatorPosition;
        this.desiredNewIntakePosition = desiredNewIntakePosition;

        this.newIntakeSubsystem = newIntakeSubsystem;
        addRequirements(this.newIntakeSubsystem); // requires newIntakeSubsystem to be the only one running
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double newIntakeOutput = MathUtil.clamp(
                controller1.calculate(newIntakeSubsystem.getRotatorPosition(), desiredNewIntakePosition),
                -0.4, 0.4

        );
        // double corlIntakeOutput =
        // MathUtil.clamp(controller2.calculate(corlSubsystem.getIntakePosition(),
        // desiredIntakePosition), -0.6, 0.6);
        // double elevatorOutput =
        // MathUtil.clamp(controller3.calculate(corlSubsystem.getElevatorPosition(),
        // desiredElevatorPosition), -0.4, 0.8);
        // We defined PID controller as controller 1

        // newIntakeSubsystem.intakeRotatorNoCommand(corlIntakeOutput);
        // newIntakeSubsystem.elevatorRunNoCommand(elevatorOutput);

        // newIntakeSubsystem.rotatorNoCommand(newIntakeOutput);

        SmartDashboard.putNumber("INTAKE POS", newIntakeSubsystem.getRotatorPosition());

        // if(newIntakeSubsystem.getElevatorPosition()<40)
        // {
        // newIntakeSubsystem.runRotatorNoCommand(corlRotatorOutput);
        // }
        // else{
        // newIntakeSubsystem.runRotatorNoCommand(0);
        // }

        // If the rotator is -25 or greater and the elevator is not below a certain
        // height

        // if(corlSubsystem.getElevatorPosition() >=65 && corlRotatorOutput>0){
        // corlSubsystem.runRotatorNoCommand(0);
        // }
        // else{
        // }

        // if(corlSubsystem.getElevatorPosition()>-40 ||
        // (corlSubsystem.getRotatorPosition()>=-50&&corlRotatorOutput>0)||(corlSubsystem.getRotatorPosition()<=0&&corlRotatorOutput<0)){
        // }
        // corlSubsystem.runRotatorNoCommand(elevatorOutput);

        // corlSubsystem.intakeRotate(corlIntakeOutput);
        // corlSubsystem.runElevator(elevatorOutput);

        newIntakeSubsystem.runRotator(newIntakeOutput);

    }

    @Override
    public boolean isFinished() {
        return controller1.atSetpoint();
    }

    // @Override
    // public void end(boolean interrupted){
    // newIntakeSubsystem.HaltRotator();
    // }
}
