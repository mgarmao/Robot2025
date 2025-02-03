package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Algae_new extends SubsystemBase {
    // wrist
    private SparkMax wMotor1;
    private SparkMax wMotor2;
    private SparkMaxConfig wMotor1_conf;
    private SparkMaxConfig wMotor2_conf;
    private RelativeEncoder wMotor1_enc;

    public Algae_new() {
        wMotor1 = new SparkMax(Constants.Motors.WRIST_MOTOR_1, MotorType.kBrushless);
        wMotor1_conf = new SparkMaxConfig();
        wMotor1_enc = wMotor1.getEncoder(); // !! RELATIVE ENCODER. START WRIST AT UP POSITION. !!

        wMotor1_conf
            .idleMode(IdleMode.kBrake);
        
        wMotor2 = new SparkMax(Constants.Motors.WRIST_MOTOR_2, MotorType.kBrushless);
        wMotor2_conf = new SparkMaxConfig();

        wMotor2_conf
            .idleMode(IdleMode.kBrake)
            .follow(Constants.Motors.WRIST_MOTOR_1)
            .inverted(true);

        wMotor1.configure(wMotor1_conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wMotor2.configure(wMotor2_conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command wristUp() {
        return runOnce(
            () -> {
                wMotor1.set(.8);
            }
        );
    }

    public Command wristDown() {
        return runOnce(
            () -> {
                wMotor1.set(-.8);
            }
        );
    }

    public Command stopWrist() {
        return runOnce(
            () -> {
                wMotor1.set(0);
                
            }
        );
    }

    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("wristPos", wMotor1_enc.getPosition());
    }
}
