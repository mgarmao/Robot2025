// package frc.robot.subsystems;

// // coral shitter or something i genuinely don't know
// // made by yours truly ruben
// // mb mike i took a lil long, but i did it so we good.

// // package frc.robot.subsystems;

// // import edu.wpi.first.wpilibj2.command.Command;
// // import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.spark.*;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import frc.robot.Constants;

// public class Corl extends SubsystemBase {
//     private SparkMaxConfig motorConfig;
//     private SparkMax motor;

//     public Corl() {
//         motor = new SparkMax(Constants.Motors.CORL_MOTOR, MotorType.kBrushless);
//         motorConfig = new SparkMaxConfig();    
        
//         motorConfig
//             .idleMode(IdleMode.kBrake);
        
//         motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     public Command corlIntake() {
//         return runOnce(
//             () -> {
//                 motor.set(.7);
//             });
//     }

//     public Command corlOuttake() {
//         return runOnce(
//             () -> {
//                 motor.set(-.7);
//             });
//     }

//     public Command corlStop() {
//         return runOnce(
//             () -> {
//                 motor.set(0);
//             });
//     }
// }
