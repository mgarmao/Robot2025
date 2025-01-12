// algae in/outake sys

package frc.robot.subsystems;


import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Algae extends SubsystemBase {
    private SparkMaxConfig motorConfig;
    private SparkMax motor;

    public Algae() {
        motor = new SparkMax(Constants.AlGAE_MOTOR, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();    
        
        motorConfig
            .idleMode(IdleMode.kBrake);
        
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command algaeIntake() {
        return runOnce(
            () -> {
                motor.set(.7);
            });
    }

    public Command algaeOuttake() {
        return runOnce(
            () -> {
                motor.set(-.7);
            });
    }

    public Command algaeStop() {
        return runOnce(
            () -> {
                motor.set(0);
            });
    }
}
