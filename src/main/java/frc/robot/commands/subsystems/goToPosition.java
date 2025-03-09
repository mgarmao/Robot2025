package frc.robot.commands.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;

public class goToPosition extends Command{
    private final Algae ALGAE;
  final         CommandXboxController oppXbox = new CommandXboxController(1);

    private PIDController pidController1 = new PIDController(0.03, 0, 0);
    double desiredPosition;

    double speed;

    public goToPosition(double desiredPosition, Algae ALGAE, double speed){
        this.ALGAE = ALGAE;
        this.desiredPosition = desiredPosition;
        this.speed = speed;
        addRequirements(this.ALGAE);
    }

    @Override
    public void initialize() {
        // Initialization code if needed
    }

    @Override
    public void execute() {
        double position = ALGAE.getRotatorPosition();

        double output = pidController1.calculate(position, desiredPosition);
        if(output>0.6){
            output = 0.6;
        }
        if(output<-0.6){
            output=-0.6;
        }
        SmartDashboard.putNumber("rotator PID", output);
        ALGAE.runWheels(speed);
        ALGAE.runRotator(output);

        // if (ALGAE.intakeWheelVoltage() <= 4 ) {
        //     oppXbox.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.9);
        // } else {
        //     oppXbox.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
        // }

    }

    @Override
    public boolean isFinished() {
        return ALGAE.getRotatorPosition() == desiredPosition;
    }

    @Override
    public void end(boolean interrupted) {
        ALGAE.moveRotator(0.0);
        ALGAE.runWheels(0.03);
        oppXbox.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
    }
}
