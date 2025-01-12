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


public class Wrist extends SubsystemBase{
    private SparkMax motor;
    private SparkMaxConfig config;

    public Wrist() {
        motor = new SparkMax(Constants.WRIST_MOTOR, MotorType.kBrushless);
        config = new SparkMaxConfig();

        config.
            idleMode(IdleMode.kBrake);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    

    // highkey could be reversed but who rly gives a shit rn it hasnt been made yet
    public Command wristUp() { return runOnce(() -> { motor.set(.6); } ); }

    public Command wristDown() { return runOnce(() -> { motor.set(-.6); } ); }

    public Command wristStop() { return runOnce(() -> { motor.set(0); } ); }
}
