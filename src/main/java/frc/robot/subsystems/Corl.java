package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;

public class Corl extends SubsystemBase {
    private SparkMax intakeWheels;
    private SparkMaxConfig intakeWheelsConfig;
    
    private SparkMax intakeRotator;
    private SparkMaxConfig intakeRotatorConfig;

    private TalonFX rotator_motor1 = new TalonFX(Constants.Motors.ROTATOR_LEFT_MOTOR);
    private TalonFX rotator_motor2 = new TalonFX(Constants.Motors.ROTATOR_RIGHT_MOTOR);

    private TalonFX elevatorMotor1 = new TalonFX(Constants.Motors.ELEVATOR_LEFT);
    private TalonFX elevatorMotor2 = new TalonFX(Constants.Motors.ELEVATOR_RIGHT);


    private PIDController pidController1 = new PIDController(0.5, 0, 0);

    public Corl() {
        intakeWheels = new SparkMax(Constants.Motors.CORL_MOTOR, MotorType.kBrushless);
        intakeWheelsConfig = new SparkMaxConfig();  
        intakeWheelsConfig.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(Constants.CurrentLimits.intakeWheels);
        intakeWheels.configure(intakeWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeRotator = new SparkMax(Constants.Motors.WRIST_MOTOR, MotorType.kBrushless);
        intakeRotatorConfig = new SparkMaxConfig();
        intakeRotatorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.CurrentLimits.intakeRotator);
        intakeRotator.configure(intakeRotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        CurrentLimitsConfigs configs1 = new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(20).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        rotator_motor1.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(configs1));
        
        CurrentLimitsConfigs configs2 = new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(20).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        rotator_motor2.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(configs2));

        rotator_motor1.setNeutralMode(NeutralModeValue.Brake);
        rotator_motor2.setNeutralMode(NeutralModeValue.Brake);
        rotator_motor1.setInverted(false);
        rotator_motor2.setInverted(true);

        CurrentLimitsConfigs elevatorConfigs1 = new CurrentLimitsConfigs().withStatorCurrentLimit(120).withSupplyCurrentLimit(Constants.CurrentLimits.elevator).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        elevatorMotor1.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(elevatorConfigs1));
        CurrentLimitsConfigs elevatorConfigs2 = new CurrentLimitsConfigs().withStatorCurrentLimit(120).withSupplyCurrentLimit(Constants.CurrentLimits.elevator).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        elevatorMotor2.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(elevatorConfigs2));
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor1.setInverted(true);
        elevatorMotor2.setInverted(false);

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

    public Command runRotator(double speed) {
        return runOnce(
            () -> {
                rotator_motor1.set(speed);
                rotator_motor2.set(speed);
            });
    }

    public Command moveRotatorToPosition(double desiredPosition) {
        return runOnce(
            () -> {
                double position = rotator_motor1.getPosition().getValueAsDouble();

                double output = pidController1.calculate(position, desiredPosition);
                rotator_motor1.set(output);
                rotator_motor2.set(output);

            });
    }
    
    public Command moveIntakeToPosition(double desiredPosition) {
        return runOnce(
            () -> {
                double positon = intakeRotator.getEncoder().getPosition();

                double output = pidController1.calculate(positon, desiredPosition);
                intakeRotator.set(output);
            });
    }

    public Command intakeRotate(double speed) {
        return runOnce(
            () -> {
                intakeRotator.set(speed);
            });
    }

    public Command runIntake(double speed){
        return runOnce(
        () -> {
            intakeWheels.set(speed);
            SmartDashboard.putNumber("Intake Bus Voltage", intakeWheels.getBusVoltage());
        });
    }

    public Command runElevator(double speed){
        return runOnce(
        () -> {
            elevatorMotor1.set(speed);
            elevatorMotor2.set(speed);
        });
    }

    public double getRotatorPosition(){
        return rotator_motor1.getPosition().getValueAsDouble();
    }

    public double getIntakePosition(){
        return intakeRotator.getEncoder().getPosition();
    }
}
