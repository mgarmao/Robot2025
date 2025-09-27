package frc.robot.commands.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewIntake;

public class GoSetpoint extends Command {
    PIDController controller1 = new PIDController(0.15, 0.04, 0.0);
    private NewIntake newIntake;
    double desiredRotatorPosition;

    public GoSetpoint(double desiredRotatorPosition, NewIntake newIntake) {
        this.desiredRotatorPosition = desiredRotatorPosition;
        this.newIntake = newIntake;
        addRequirements(this.newIntake);
    }

    @Override
    public void execute() {
        double intakeRotatorOut = MathUtil.clamp(controller1.calculate(newIntake.ReturnRotatorPosition()), -0.8d, 0.8d);
    
        newIntake.rotatorNoCommand(intakeRotatorOut);

        SmartDashboard.putNumber("Intake Rotator Pos", intakeRotatorOut);
    }
}
