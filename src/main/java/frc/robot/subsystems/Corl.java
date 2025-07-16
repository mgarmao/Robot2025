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
//Mr. Sisodia
        /*Mike should do the gnarly dance */
    // How to make a new motor: 
        // 1. Configure motor's ID (35-36)
        // 2. Create and Apply a limit configuration(65)
        // 3. Set the neutral mode (77-78)

import frc.robot.Constants;

public class Corl extends SubsystemBase {
    private SparkMax intakeWheels;
    private SparkMaxConfig intakeWheelsConfig;

    private SparkMax intakeRotator;
    private SparkMaxConfig intakeRotatorConfig; // First motor rotator, for the company, "Spark Max"

    private SparkMax flagMotor; 
    private SparkMaxConfig flagMotorConfig; // First motor rotator, for the company, "Spark Max"

    private SparkMax chargeMotor;
    private SparkMaxConfig chargeMotorConfig; // First motor rotator, for the company, "Spark Max"

    private SparkMax motator;
    private SparkMaxConfig motatorConfig; // First motor rotator, for the company, "Spark Max" (not used in this code, but can be used later on) 

    private TalonFX rotator_motor1 = new TalonFX(Constants.Motors.ROTATOR_LEFT_MOTOR);
    private TalonFX rotator_motor2 = new TalonFX(Constants.Motors.ROTATOR_RIGHT_MOTOR); //Second motor rotator for the company "TalonFX" 

    // private TalonFX elevatorMotor1 = new TalonFX(Constants.Motors.ELEVATOR_LEFT);
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

        flagMotor = new SparkMax(Constants.Motors.flag_motor, MotorType.kBrushless);
        flagMotorConfig = new SparkMaxConfig(); 
        flagMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        flagMotor.configure(flagMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        motator = new SparkMax(Constants.Motors.motator, MotorType.kBrushless);
        motatorConfig = new SparkMaxConfig(); 
        motatorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        motator.configure(motatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turboMotor = new TalonFX(Constants.Motors.turboMotor);
        TalonFXConfiguration turbomotorConfig = new TalonFXConfiguration();
        turboMotor.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        turboMotor.configure(turboMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        chargeMotor = new SparkMax(Constants.Motors.chargeMotor, MotorType.kBrushless); // all motors are kBrushless this line initializes the charge motor
        chargeMotorConfig = new SparkMaxConfig(); // company name "Spark Max"
        chargeMotorConfig.inverted(false) // set the motor to not inverted
                         .idleMode(IdleMode.kBrake) // settings 
                         .smartCurrentLimit(Constants.CurrentLimits.chargeMotor); // settings
        chargeMotor.configure(chargeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        // Implement Energy Efficiency Mode
        SmartDashboard.putNumber("Charge Motor Speed", 0.0);
    
    
    
    
        
        MotorOutputConfigs inverConfig = new MotorOutputConfigs().withInverted(Constants.InvertedEnum.Clockwise);

        CurrentLimitsConfigs limitConfigs1 = new CurrentLimitsConfigs().withStatorCurrentLimit(80).withSupplyCurrentLimit(30).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        rotator_motor1.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(limitConfigs1));
        
        CurrentLimitsConfigs limitConfigs2 = new CurrentLimitsConfigs().withStatorCurrentLimit(80).withSupplyCurrentLimit(30).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        rotator_motor2.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(limitConfigs2).withMotorOutput(inverConfig));

        rotator_motor1.setNeutralMode(NeutralModeValue.Brake);
        rotator_motor2.setNeutralMode(NeutralModeValue.Brake);


        // CurrentLimitsConfigs elevatorConfigs1 = new CurrentLimitsConfigs().withStatorCurrentLimit(120).withSupplyCurrentLimit(Constants.CurrentLimits.elevator).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        // elevatorMotor1.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(elevatorConfigs1).withMotorOutput(inverConfig));
        // elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        // elevatorMotor1.setInverted(true);
        CurrentLimitsConfigs elevatorConfigs2 = new CurrentLimitsConfigs().withStatorCurrentLimit(120).withSupplyCurrentLimit(Constants.CurrentLimits.elevator).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);
        elevatorMotor2.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(elevatorConfigs2));
        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2.setInverted(false);

        // elevatorMotor1.getConfigurator().apply(slot0Configs);
        

        SignalLogger.enableAutoLogging(false);
    }

    public Command armUp() {
        return runOnce(
            () -> {
                rotator_motor1.set(1);
                rotator_motor2.set(1);
            });
    }

    public Command raiseTheFlag(double speed) {
        return runOnce(
            () -> {
                double clampSpeed = clamp(speed, -1, 1); 
                flagMotor.set(clampSpeed);
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
                double position = rotator_motor2.getPosition().getValueAsDouble();

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
            // SmartDashboard.putNumber("Elevator1 Positon", elevatorMotor1.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator2 Positon", elevatorMotor2.getPosition().getValueAsDouble());
            // elevatorMotor1.set(speed);
            if((getRotatorPosition()<5||getRotatorPosition()>-48||speed<0)&&(getElevatorPosition()<=80||speed<0)){
                elevatorMotor2.set(speed);
            }
            else{
                elevatorMotor2.set(0);
            }
        });
    }

    public Command runMotator(double desiredPosition){
        return runOnce(
        () -> {
            double positon = motator.getEncoder().getPosition();

            double output = clamp(pidController1.calculate(positon, desiredPosition), 0.3, -0.3);
            motator.set(output);
        });
    }

    public double getRotatorPosition(){
        return rotator_motor2.getPosition().getValueAsDouble();
    }

    public double getIntakePosition(){
        return intakeRotator.getEncoder().getPosition();
    }

    public double getElevatorPosition(){
        return elevatorMotor2.getPosition().getValueAsDouble();
    }

    public Command elevatorGoToPosition(double desiredPosition) {
        return runOnce(
            () -> {
                // double output1 = clamp(pidController2.calculate(elevatorMotor1.getPosition().getValueAsDouble(), desiredPosition), 0.4,-0.4);
                double output2 = clamp(pidController2.calculate(elevatorMotor2.getPosition().getValueAsDouble(), desiredPosition), 0.4,-0.4);
                
                // elevatorMotor1.set(output1);
                elevatorMotor2.set(output2);

                SmartDashboard.putNumber("OUTPUT2", output2);
            });
    }

    public Command armGoToPosition(double desiredPosition) {
        return runOnce(
            () -> {
                double output1 = clamp(pidController3.calculate(rotator_motor1.getPosition().getValueAsDouble(), desiredPosition), -0.4,0.4); 
        
                double output2 = clamp(pidController3.calculate(rotator_motor2.getPosition().getValueAsDouble(), desiredPosition),-0.4,0.4);
                //  Double data type for 0.4 and 0.4, clamp means that the speed doesn't go over 40% or under 40% (backwards). 
                
                rotator_motor1.set(output1);
                rotator_motor2.set(output2);
            });
    } 
    public Command setchargeMotorSpeed(double speed) {
        return runOnce(
            () -> {
                double batteryVoltage = SmartDashboard.getNumber("Battery Voltage", 12.0); // Example battery voltage
                double efficiencyFactor = batteryVoltage / 12.0; // Scale speed based on battery voltage
                double adjustedSpeed = speed * efficiencyFactor; // Adjust speed for efficiency
                chargeMotor.set(adjustedSpeed); 
        
            SmartDashboard.putNumber("Charge Motor Speed", adjustedSpeed); // Prints if battery is low, depending on the situation
        });
    }

    public void runRotatorNoCommand(double speed){
        rotator_motor1.set(speed);
        rotator_motor2.set(speed);
    }

    public void intakeRotatorNoCommand(double speed){
        intakeRotator.set(speed);
    }

    public void elevatorRunNoCommand(double speed){
        // elevatorMotor1.set(speed);
        if((getRotatorPosition()<5||getRotatorPosition()>-48||speed<0)&&(getElevatorPosition()<=80||speed<0)){
            elevatorMotor2.set(speed);
        }        
        else{
            elevatorMotor2.set(0);   
        }
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

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Arm Position", rotator_motor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist Position", intakeRotator.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Now Position", elevatorMotor2.getPosition().getValueAsDouble());        
    }
    
}


