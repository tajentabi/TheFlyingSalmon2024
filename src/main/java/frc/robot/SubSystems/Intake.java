// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.takeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Outtake. */
  CANSparkMax intakeMotor = new CANSparkMax(takeConstants.INTAKE_ID, MotorType.kBrushless);
  DigitalInput limitSwtich = new DigitalInput(0);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(20);
    intakeMotor.enableVoltageCompensation(10);
    intakeMotor.burnFlash();
  }
  
  public boolean isNoteTouching() {
    return limitSwtich.get();
  }

  public void Run(double speed){
    speed = MathUtil.clamp(speed, -1, 1);
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
