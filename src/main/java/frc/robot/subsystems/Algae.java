// algae in/outake sys

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

public class Algae extends SubsystemBase {
    private SparkMaxConfig motorConfig;
    private SparkMax motor; // grippy
    private SparkMaxConfig motor2Config;
    private SparkMax motor2; // wrist
    private RelativeEncoder m1Encoder;
    private RelativeEncoder m2Encoder;


    public Algae() {
        motor = new SparkMax(Constants.Motors.AlGAE_MOTOR, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();    
        m1Encoder = motor.getEncoder();

        motor2 = new SparkMax(Constants.Motors.WRIST_MOTOR, MotorType.kBrushless);
        motor2Config = new SparkMaxConfig();
        m2Encoder = motor2.getEncoder();



        motorConfig
            .idleMode(IdleMode.kBrake);
        
        motor2Config
            .idleMode(IdleMode.kBrake);
            

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command algaeIntake() {
        return runOnce(
            () -> {
                motor.set(.7);
            });
    }

    public Command algaeOuttake() {
        return runOnce(
            () -> {
                motor.set(-.7);
            });
    }

    public Command shaurya() {return runOnce(() -> {Commands.print("shauryasosigma72");});}

    public Command wristUp() {return runOnce(() -> {motor2.set(.6);});} 
    public Command wristDown() {return runOnce(() -> {motor2.set(-.6);});} // confusing how this line is shorter than line 64 but hey, who cares?
    
    public Command stop() {return runOnce(() -> {motor2.set(0);motor.set(0);});}

    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("AlgaeMotorVel", m1Encoder.getVelocity());
        SmartDashboard.putNumber("WristMotorPos", m2Encoder.getPosition());
    }
}
