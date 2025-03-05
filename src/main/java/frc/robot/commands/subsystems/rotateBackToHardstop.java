package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;

public class rotateBackToHardstop extends Command{
    private final Algae ALGAE;

    public rotateBackToHardstop(Algae ALGAE){
        this.ALGAE = ALGAE;
        addRequirements(this.ALGAE);
    }

    @Override
    public void initialize() {
        // Initialization code if needed
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Rotator Position", ALGAE.getRotatorPosition());
        if(ALGAE.getRotatorPosition()<=Constants.Setpoints.AlgaeIn-10){
            ALGAE.runRotator(0.7);
            ALGAE.runWheels(0.7);
        }
        else{
            ALGAE.runRotator(0.5);
            ALGAE.runWheels(0.4);
        }

    }

    @Override
    public boolean isFinished() {
        return ALGAE.getRotatorPosition() >= Constants.Setpoints.AlgaeIn-2;
    }

    @Override
    public void end(boolean interrupted) {
        ALGAE.moveRotator(0.0);
        ALGAE.runWheels(0.03);
    }
}
