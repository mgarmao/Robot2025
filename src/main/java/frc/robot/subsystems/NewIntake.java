package frc.robot.subsystems;
// WPI Required Stuff
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// REVLib
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// Phoenix6
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class NewIntake extends SubsystemBase {
    private SparkMax Spinny;
    private SparkMaxConfig SpinnyConf;
    private TalonFX Rotator;
    private TalonFX NonRotator;
    private boolean ifSpinnyOut = false;

    public NewIntake() {
        Spinny = new SparkMax(Constants.Motors.SpinnyMotor, MotorType.kBrushless);
        SpinnyConf = new SparkMaxConfig();

        SpinnyConf
            .inverted(false) // change if wrong
            .idleMode(IdleMode.kBrake);
        Spinny.configure(SpinnyConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

    public Command Intake(boolean in) {
        if (in) {
            return runOnce( () -> {
                Spinny.set(.38d);
                ifSpinnyOut = false;
            });
        } else {
            return runOnce( () -> {
                Spinny.set(-.67d);
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
}