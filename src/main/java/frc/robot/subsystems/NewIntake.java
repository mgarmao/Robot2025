package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// WPI Required Stuff
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
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
                        .withInverted(InvertedValue.CounterClockwise_Positive)
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

         private PIDController pidController3 = new PIDController(1, 0, 0);

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

    public Command runRotator(double speed) {
        return runOnce(
            () -> {
                Rotator.set(speed);
                NonRotator.set(speed);
            });
    }

    public Command Rotate_Goto(double desiredPosition) {
        return runOnce(() -> {
                double output1 = MathUtil.clamp(pidController3.calculate(Rotator.getPosition().getValueAsDouble(), desiredPosition), -0.4,0.4); 
        
                double output2 = MathUtil.clamp(pidController3.calculate(NonRotator.getPosition().getValueAsDouble(), desiredPosition),-0.4,0.4);
                 //  Double data type for 0.4 and 0.4, clamp means that the speed doesn't go over 40% or under 40% (backwards). 
                
                 Rotator.set(output1);
                 NonRotator.set(output2);

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

    public Command HaltRotator() {
        return runOnce(() -> {Rotator.stopMotor(); NonRotator.stopMotor();});
    }

    public void rotatorNoCommand (double speed) {
        Rotator.set(speed);
        NonRotator.set(speed);
    }

    public double getRotatorPosition() {
        return Rotator.getPosition().getValueAsDouble();
    }

    public double getNonRotatorPosition() {
        return NonRotator.getPosition().getValueAsDouble();
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("Rotator Position", Rotator.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("NonRotator Position", NonRotator.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Spinny Speed", SpinnyEnc.getVelocity());
    }

    public static Command GoToSetpoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GoToSetpoint'");
    }  
}