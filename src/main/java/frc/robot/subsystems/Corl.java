package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;

public class Corl extends SubsystemBase {

    private TalonFX rotator_motor1 = new TalonFX(Constants.Motors.ROTATOR_LEFT_MOTOR);
    private TalonFX rotator_motor2 = new TalonFX(Constants.Motors.ROTATOR_RIGHT_MOTOR);

    private TalonFX intake_rotator = new TalonFX(Constants.Motors.ROTATOR_LEFT_MOTOR);

    private PIDController pidController1 = new PIDController(0.5, 0, 0);

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

    public Command moveRotatorToPosition(double desiredPosition) {
        return runOnce(
            () -> {
                double positon = rotator_motor1.getPosition().getValueAsDouble();

                double output = pidController1.calculate(positon, desiredPosition);
                rotator_motor1.set(output);
                rotator_motor2.set(output);

            });
    }
    
    public Command moveIntakeToPosition(double desiredPosition) {
        return runOnce(
            () -> {
                double positon = intake_rotator.getPosition().getValueAsDouble();

                double output = pidController1.calculate(positon, desiredPosition);
                intake_rotator.set(output);
                intake_rotator.set(output);

            });
    }

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
