package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// WPI Required Stuff
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
// REVLib
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
// Phoenix6
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class NewIntake extends SubsystemBase {
    private SparkMax Spinny;
    private SparkMaxConfig SpinnyConf;
    private RelativeEncoder SpinnyEnc;
    private TalonFX Rotator;
    private TalonFX NonRotator;
    private boolean ifSpinnyOut = false;
    private Slot0Configs slot0 = new Slot0Configs();
    private final PositionVoltage PV = new PositionVoltage(0).withSlot(0);

    public NewIntake() {
        Spinny = new SparkMax(Constants.Motors.SpinnyMotor, MotorType.kBrushless);
        SpinnyConf = new SparkMaxConfig();

        SpinnyConf
            .inverted(false) // change if wrong
            .idleMode(IdleMode.kBrake);
        Spinny.configure(SpinnyConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SpinnyEnc = Spinny.getEncoder();

        slot0.kP = 2.4;
        slot0.kI = 0.0;
        slot0.kD = 0.1;

        Rotator = new TalonFX(6); // ROTATOR_RIGHT_MOTOR
        Rotator.getConfigurator().apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)
                )
        );

        NonRotator = new TalonFX(5); // ROTATOR_LEFT_MOTOR
        NonRotator.getConfigurator().apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        // .withInverted(null)
                        .withNeutralMode(NeutralModeValue.Coast)
                )
        );

        // keep spinny in constant rotation
        Spinny.set(.04d);

        if (Rotator.getPosition().getValueAsDouble() > Constants.setpoint_Rotator_Default_StartRange && Rotator.getPosition().getValueAsDouble() < Constants.setpoint_Rotator_Default_EndRange)
            {
                Rotator.setPosition(0d);
            }
    }

    public Command Rotate(boolean up) {
        if (up) {
            return runOnce( () -> {
                Rotator.set(.35d);
                NonRotator.set(.35d);
            });
        } else {
            return runOnce( () -> {
                Rotator.set(-.35d);
                NonRotator.set(-.35d);
            });
        }
    }

    public Command Rotate_Goto(int pos) {

        // 0 = Default
        // 1 = Floor
        // 2 = Shoot
        return runOnce( () -> {
            if (pos == 0) {

                // do the math
                // double result = Constants
                
                // Rotator.setPosition()+
            }
        });
    }

    public Command Intake(boolean in) {
        if (in) {
            return runOnce( () -> {
                Spinny.set(.38d);
                ifSpinnyOut = false;
            });
        } else {
            return runOnce( () -> {
                Spinny.set(-1.d);
                ifSpinnyOut = true;
            });
        }
    }

    public Command HaltIntake() {
        return runOnce(() -> {
            if (ifSpinnyOut) {
                Spinny.set(-.04d);
            } else {
                Spinny.set(.04d);
            }
        });
    }

    public void rotatorNoCommand(double speed) {
        Rotator.set(speed);
        NonRotator.set(speed);
    }

    public double ReturnRotatorPosition() {
        return Rotator.getPosition().getValueAsDouble();
    }

    public Command HaltRotator() {
        return runOnce(() -> {Rotator.stopMotor(); NonRotator.stopMotor();});
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Rotator Position", Rotator.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("NonRotator Position", NonRotator.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Spinny Speed", SpinnyEnc.getVelocity());
    }  
}