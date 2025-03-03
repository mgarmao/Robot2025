package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Algae extends SubsystemBase {
    private TalonFX rotatorMotor = new TalonFX(Constants.Motors.AlGAE_ROTATOR);
    private TalonFX intakeWheels = new TalonFX(Constants.Motors.AlGAE_INTAKE);
    private PIDController pidController1 = new PIDController(0.02, 0, 0);

    public Algae() {
        CurrentLimitsConfigs configs1 = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimit(Constants.CurrentLimits.algaeRotator)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        TalonFXConfiguration rotatorConfig = new TalonFXConfiguration();
        rotatorConfig.withCurrentLimits(configs1);
        rotatorConfig.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(0)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(-54)
                .withReverseSoftLimitEnable(true)
        );
        
        rotatorMotor.getConfigurator().apply(rotatorConfig);

        CurrentLimitsConfigs configs2 = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimit(Constants.CurrentLimits.algaeIntakeWheels)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        intakeWheels.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(configs2));
    }

    public Command moveRotator(double speed) {
        return runOnce(
            () -> {
                rotatorMotor.set(speed);
                SmartDashboard.putNumber("rotator Pose", rotatorMotor.getPosition().getValueAsDouble());
            }
        );
    }

    public void runRotator(double speed) {
        rotatorMotor.set(speed);
    }

    public void runWheels(double speed) {
        intakeWheels.set(speed);
    }
        
    

    public Command runIntake(double speed) {
        return runOnce(
            () -> {
                intakeWheels.set(speed);
            }
        );
    }

    public Command smartRunIntake(double speed) {
        return runOnce(
            () -> {
                if(intakeWheels.getMotorVoltage().getValueAsDouble()>=6.4){
                    intakeWheels.set(0.05);
                }
                else{
                    intakeWheels.set(speed);
                }
            }
        );
    }

    public Command rotatorToPosition(double desiredPosition) {
        return runOnce(
            () -> {
               
            }
        );
    }

    public Command rotateBackToHardstop() {
        return run(
            () -> {
                SmartDashboard.putNumber("Rotator Position", rotatorMotor.getPosition().getValueAsDouble());
                if(rotatorMotor.getPosition().getValueAsDouble()<=Constants.Setpoints.AlgaeIn){
                    rotatorMotor.set(0.7);
                    intakeWheels.set(0.7);
                }
                else{
                    rotatorMotor.set(0.0);
                    intakeWheels.set(0.03);
                }
            }
        );
    }

    public double getRotatorPosition(){
        return rotatorMotor.getPosition().getValueAsDouble();
    }

    public double intakeWheelVoltage(){
        return intakeWheels.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Rotator Position", getRotatorPosition());
        SmartDashboard.putNumber("INTAKE WHEEL Vs", intakeWheels.getMotorVoltage().getValueAsDouble());

        
    }

}

