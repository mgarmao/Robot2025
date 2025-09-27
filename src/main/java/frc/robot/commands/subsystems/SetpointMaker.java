package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewIntake;

public class SetpointMaker extends Command {
    private final NewIntake intake;

    public SetpointMaker(NewIntake intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        double rot = intake.ReturnRotatorPosition();
        System.out.println(rot + " :: RECENTLY SAVED SETPOS");
    }

    @Override
    public boolean isFinished() {
        return false; // keep running until cancelled
    }
}
