package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Algae extends SubsystemBase {
    private TalonFX rotatorMotor = new TalonFX(Constants.Motors.AlGAE_ROTATOR);
    private TalonFX intakeWheels = new TalonFX(Constants.Motors.AlGAE_INTAKE);
    private PIDController pidController1 = new PIDController(0.5, 0, 0);

    public Algae() {
        CurrentLimitsConfigs configs1 = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.CurrentLimits.algaeRotator)
            .withSupplyCurrentLimit(Constants.CurrentLimits.algaeRotator)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        rotatorMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(configs1));

        CurrentLimitsConfigs configs2 = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.CurrentLimits.algaeIntakeWheels)
            .withSupplyCurrentLimit(Constants.CurrentLimits.algaeIntakeWheels)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        intakeWheels.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(configs2));
    }

    public Command moveRotator(double speed) {
        return runOnce(
            () -> {
                rotatorMotor.set(speed);
            }
        );
    }

    public Command runIntake(double speed) {
        return runOnce(
            () -> {
                intakeWheels.set(speed);
            }
        );
    }

    public Command rotatorToPosition(double desiredPosition) {
        return runOnce(
            () -> {
                double position = rotatorMotor.getPosition().getValueAsDouble();

                double output = pidController1.calculate(position, desiredPosition);
                rotatorMotor.set(output);
            }
        );
    }
}
