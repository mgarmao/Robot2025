// algae in/outake sys

package frc.robot.subsystems;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Algae extends SubsystemBase {
    private SparkMaxConfig motorConfig;
    private SparkMaxConfig motorConfig2;
    private SparkMax motor;
    private SparkMax motor2;

    public Algae() {
        motor = new SparkMax(Constants.AlGAE_MOTOR, MotorType.kBrushless);
        motor2 = new SparkMax(Constants.AlGAE_MOTOR2, MotorType.kBrushless);    
        motorConfig = new SparkMaxConfig();    
        
        motorConfig
            .idleMode(IdleMode.kBrake);
        
        motorConfig2
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motorConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command algaeIntake() {
        return runOnce(
            () -> {
                motor.set(.7);
                motor2.set(.7);
            });
    }

    public Command algaeOuttake() {
        return runOnce(
            () -> {
                motor.set(-.7);
                motor2.set(-.7);
            });
    }
}
