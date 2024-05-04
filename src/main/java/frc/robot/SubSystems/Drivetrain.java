// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.SubSystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.TalonFXConstants;
// import frc.robot.Constants.OperatorConstants;
// import edu.wpi.first.math.MathUtil;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj.GenericHID;

// public class Drivetrain extends SubsystemBase {
//     private final TalonFX leftL = new TalonFX(TalonFXConstants.LEFT_LEADER_ID);
//     private final TalonFX leftF = new TalonFX(TalonFXConstants.LEFT_FOLLOWER_ID);
//     private final TalonFX rightL = new TalonFX(TalonFXConstants.RIGHT_LEADER_ID);
//     private final TalonFX rightF = new TalonFX(TalonFXConstants.RIGHT_FOLLOWER_ID);
//     private final DifferentialDrive m_drive = new DifferentialDrive(leftL, rightL);
//     private CommandXboxController controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);
//     double throttleValue = 0.5;
//     double rotThrottle = 0.5;
//     boolean upListening = true;
//     boolean rightListening = true;
//     boolean leftListening = true;
//     boolean downListening = true;

//     public Drivetrain() {
//         var leftConfiguration = new TalonFXConfiguration();
//         var rightConfiguration = new TalonFXConfiguration();
//         leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
//         rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//         leftL.getConfigurator().apply(leftConfiguration);
//         leftF.getConfigurator().apply(leftConfiguration);
//         rightL.getConfigurator().apply(rightConfiguration);
//         rightF.getConfigurator().apply(rightConfiguration);
//         leftF.setControl(new Follower(leftL.getDeviceID(), false));
//         rightF.setControl(new Follower(rightL.getDeviceID(), false));
//         leftL.setSafetyEnabled(true);
//         rightL.setSafetyEnabled(true);
//     }

//     public void ControllerRun() {
//         // Thought Id keep the old code just in case ;)
//         GenericHID cont = controller.getHID();
//         // if (cont.getPOV() == 0) {
//         //     if (upListening) {
//         //         throttleValue += 0.1;
//         //         upListening = false;
//         //         System.out.println("[FTU] FSPEED THROTTLE: " + throttleValue);
//         //     }
//         // } else {
//         //     upListening = true;
//         // }
//         // if (cont.getPOV() == 180) {
//         //     if (downListening) {
//         //         throttleValue -= 0.1;
//         //         downListening = false;
//         //         System.out.println("[FTU] FSPEED THROTTLE: " + throttleValue);
//         //     }
//         // } else {
//         //     downListening = true;
//         // }
//         // if (cont.getPOV() == 270) {
//         //     if (leftListening) {
//         //         rotThrottle += 0.1;
//         //         leftListening = false;
//         //         System.out.println("[RTU] ROTATION THROTTLE: " + rotThrottle);
//         //     }
//         // } else {
//         //     leftListening = true;
//         // }
//         // if (cont.getPOV() == 90) {
//         //     if (rightListening) {
//         //         rotThrottle -= 0.1;
//         //         rightListening = false;
//         //         System.out.println("[RTU] ROTATION THROTTLE: " + rotThrottle);
//         //     }
//         // } else {
//         //     rightListening = true;
//         // }
//         if (cont.getPOV() == 0) {
//             throttleValue += 0.2;
//             rotThrottle += 0.2;
//             upListening = false;
//             System.out.println("[FSPEED] THROTTLE: " + throttleValue);
//         } else {
//             upListening = true;
//         }
//         if (cont.getPOV() == 270) {
//             throttleValue -= 0.2;
//             rotThrottle -= 0.2;
//             downListening = false;
//             System.out.println("[FSPEED] THROTTLE: " + throttleValue);
//         } else {
//             downListening = true;
//         }
//         throttleValue = MathUtil.clamp(throttleValue, -1, 1);
//         rotThrottle = MathUtil.clamp(rotThrottle, -1, 1);
//         m_drive.arcadeDrive((curveSensitivity(controller.getLeftY()) * throttleValue), (-controller.getRightX() * rotThrottle));
//     }
    
//     private double curveSensitivity(double val) {
//         return val * Math.abs(val);
//     }

//     public void customRun(double speed, double rot) {
//         m_drive.arcadeDrive(speed, rot);
//     }
// }