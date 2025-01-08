// coral shitter or something i genuinely don't know
// made by yours truly ruben

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Corl extends SubsystemBase {

    public Corl() {}

    public Command corlIntake() {
        return runOnce(
            () -> {
                /* one-time action goes here */
            });
    }
}