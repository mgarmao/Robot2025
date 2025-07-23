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
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

      
    // How to make a new motor: 
        // 1. Configure motor's ID (35-36)
        // 2. Create and Apply a limit configuration(65)
        // 3. Set the neutral mode (77-78)

import frc.robot.Constants;




public class Jitgoon extends SubsystemBase{

    private SparkMax gMotor;
    private SparkMaxConfig gMotorConfig;

    private SparkMax vMotor;
    private SparkMaxConfig vMotorConfig;

    private PIDController pidController4 = new PIDController(0., 0, 0);
    private PIDController pidController5 = new PIDController(1, 0, 0);
    private PIDController pidController6 = new PIDController(1, 0, 0);

    public Jitgoon(){
        gMotor = new SparkMax(Constants.Motors.gMotor, MotorType.kBrushless);
        gMotorConfig = new SparkMaxConfig();  
        gMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.CurrentLimits.intakeWheels);
        gMotor.configure(gMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        vMotor = new SparkMax(Constants.Motors.vMotor, MotorType.kBrushless);
        vMotorConfig = new SparkMaxConfig();
        vMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.CurrentLimits.intakeRotator);
        vMotor.configure(vMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);




    }
    
}
