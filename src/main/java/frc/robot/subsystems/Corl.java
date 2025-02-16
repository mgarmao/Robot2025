package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// coral shitter or something i genuinely don't know
// made by yours truly ruben
// mb mike i took a lil long, but i did it so we good.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class Corl extends SubsystemBase {
    private SparkMaxConfig motorConfig;
    private SparkMaxConfig rotator_motor1Config;
    private SparkMaxConfig rotator_motor2Config;

    private TalonFX rotator_motor1 = new TalonFX(Constants.Motors.ROTATOR_LEFT_MOTOR);
    private TalonFX rotator_motor2 = new TalonFX(Constants.Motors.ROTATOR_RIGHT_MOTOR);

    public Corl() {
        
        CurrentLimitsConfigs configs1 = new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(20).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        rotator_motor1.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(configs1));
        
        CurrentLimitsConfigs configs2 = new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(20).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        rotator_motor2.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(configs2));

        rotator_motor1.setNeutralMode(NeutralModeValue.Brake);
        
        rotator_motor1.setInverted(false);

        rotator_motor2.setNeutralMode(NeutralModeValue.Brake);
        rotator_motor2.setInverted(true);

        SignalLogger.enableAutoLogging(false);
    }

    public Command armUp() {
        return runOnce(
            () -> {
                rotator_motor1.set(1);
                rotator_motor2.set(1);
            });
    }

    public Command armDown() {
        return runOnce(
            () -> {
                rotator_motor1.set(-1);
                rotator_motor2.set(-1);
            });
    }

    public Command armStop() {
        return runOnce(
            () -> {
                rotator_motor1.set(0.0);
                rotator_motor2.set(0.0);
            });
    }

//     public Command corlIntake() {
//         return runOnce(
//             () -> {
//                 motor.set(.7);
//             });
//     }

//     public Command corlOuttake() {
//         return runOnce(
//             () -> {
//                 motor.set(-.7);
//             });
//     }

//     public Command corlStop() {
//         return runOnce(
//             () -> {
//                 motor.set(0);
//             });
//     }
}
