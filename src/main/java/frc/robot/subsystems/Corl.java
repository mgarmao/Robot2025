package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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


    private PIDController pidController1 = new PIDController(0., 0, 0);
    private PIDController pidController2 = new PIDController(1, 0, 0);
    private PIDController pidController3 = new PIDController(1, 0, 0);

    public Corl() {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 25.5; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.2; // A velocity of 1 rps results in 0.1 V output

        intakeWheels = new SparkMax(Constants.Motors.CORL_MOTOR, MotorType.kBrushless);
        intakeWheelsConfig = new SparkMaxConfig();  
        intakeWheelsConfig.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(Constants.CurrentLimits.intakeWheels);
        intakeWheels.configure(intakeWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeRotator = new SparkMax(Constants.Motors.WRIST_MOTOR, MotorType.kBrushless);
        intakeRotatorConfig = new SparkMaxConfig();
        intakeRotatorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.CurrentLimits.intakeRotator);
        intakeRotator.configure(intakeRotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        MotorOutputConfigs inverConfig = new MotorOutputConfigs().withInverted(Constants.InvertedEnum.CounterClockwise);

        CurrentLimitsConfigs limitConfigs1 = new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(20).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        rotator_motor1.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(limitConfigs1));
        
        CurrentLimitsConfigs limitConfigs2 = new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(20).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        rotator_motor2.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(limitConfigs2).withMotorOutput(inverConfig));

        rotator_motor1.setNeutralMode(NeutralModeValue.Brake);
        rotator_motor2.setNeutralMode(NeutralModeValue.Brake);

        CurrentLimitsConfigs elevatorConfigs1 = new CurrentLimitsConfigs().withStatorCurrentLimit(120).withSupplyCurrentLimit(Constants.CurrentLimits.elevator).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        elevatorMotor1.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(elevatorConfigs1).withMotorOutput(inverConfig));
        CurrentLimitsConfigs elevatorConfigs2 = new CurrentLimitsConfigs().withStatorCurrentLimit(120).withSupplyCurrentLimit(Constants.CurrentLimits.elevator).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        elevatorMotor2.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(elevatorConfigs2));
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor1.setInverted(true);
        elevatorMotor2.setInverted(false);

        elevatorMotor1.getConfigurator().apply(slot0Configs);
        

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

                double output = clamp(pidController1.calculate(positon, desiredPosition), 0.3, -0.3);
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
            SmartDashboard.putNumber("Intake Bus Voltage", intakeWheels.getBusVoltage()); // last time didnt work with robot sim (maybe just because it was a sim)
        });
    }

    public Command runElevator(double speed){
        return runOnce(
        () -> {
            SmartDashboard.putNumber("Elevator1 Positon", elevatorMotor1.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator2 Positon", elevatorMotor2.getPosition().getValueAsDouble());
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

    public Command elevatorGoToPosition(double desiredPosition) {
        return runOnce(
            () -> {
                double output1 = clamp(pidController2.calculate(elevatorMotor1.getPosition().getValueAsDouble(), desiredPosition), 0.4,-0.4);
                double output2 = clamp(pidController2.calculate(elevatorMotor2.getPosition().getValueAsDouble(), desiredPosition), 0.4,-0.4);
                
                elevatorMotor1.set(output1);
                elevatorMotor2.set(output2);

                SmartDashboard.putNumber("OUTPUT2", output2);
            });
    }

    public Command armGoToPosition(double desiredPosition) {
        return runOnce(
            () -> {
                double output1 = clamp(pidController3.calculate(rotator_motor1.getPosition().getValueAsDouble(), desiredPosition), -0.4,0.4);
                double output2 = clamp(pidController3.calculate(rotator_motor2.getPosition().getValueAsDouble(), desiredPosition),-0.4,0.4);
                
                rotator_motor1.set(output1);
                rotator_motor2.set(output2);

            });
    } 

    public double clamp (double value, double min, double max){
        double newVal = 0;
        if(value>max){
            newVal = max;
        }
        if(value<min){
            newVal = min;
        }
        return newVal;
    }
}

