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
    private SparkMax motor; // grippy
    private SparkMaxConfig motor2Config;
    private SparkMax motor2; // wrist

    public Algae() {
        motor = new SparkMax(Constants.Motors.AlGAE_MOTOR, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();    
        
        motor2 = new SparkMax(Constants.Motors.WRIST_MOTOR, MotorType.kBrushless);
        motor2Config = new SparkMaxConfig();

        motorConfig
            .idleMode(IdleMode.kBrake);
        
        motor2Config
            .idleMode(IdleMode.kBrake);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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


    public Command wristUp() {return runOnce(() -> {motor2.set(.6);});} 
    public Command wristDown() {return runOnce(() -> {motor2.set(-.6);});} // confusing how this line is shorter than line 64 but hey, who cares?
    public Command wristStop() {return runOnce(() -> {motor2.set(0);});}
}
