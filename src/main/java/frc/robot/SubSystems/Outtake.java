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
import com.revrobotics.RelativeEncoder;

public class Outtake extends SubsystemBase {
  /** Creates a new Outtake. */
  CANSparkMax outtakeMotorL = new CANSparkMax(takeConstants.OUTAKE_LEADER_ID, MotorType.kBrushless);
  CANSparkMax outtakeMotorF = new CANSparkMax(takeConstants.OUTAKE_FOLLOWER_ID, MotorType.kBrushless);
  DigitalInput limitSwtich = new DigitalInput(0);
  RelativeEncoder encoderL = outtakeMotorL.getEncoder();
  public double shootSpeed = 1;
  private boolean shootSpeedBig = true;

  public Outtake() {
    outtakeMotorF.restoreFactoryDefaults();
    outtakeMotorF.setSmartCurrentLimit(40);
    outtakeMotorF.enableVoltageCompensation(10);
    outtakeMotorF.follow(outtakeMotorL);
    outtakeMotorF.burnFlash();
    outtakeMotorL.restoreFactoryDefaults();
    outtakeMotorL.setSmartCurrentLimit(40);
    outtakeMotorL.enableVoltageCompensation(10);
    outtakeMotorL.burnFlash();
  }

  public void printRPM() {
    System.out.println(encoderL.getVelocity());
  }

  public void setSpeed(boolean sped) {
    shootSpeedBig = sped;
  }
  
  public boolean isNoteTouching() {
    return limitSwtich.get();
  }

  public boolean isNoteNotTouching() {
    return !isNoteTouching();
  }

  public void Run(double speed){
    speed = MathUtil.clamp(speed, -1, 1);
    outtakeMotorL.set(speed);
  }

  @Override
  public void periodic() {
    if (shootSpeedBig) {
      shootSpeed = 1;
    } else {
      shootSpeed = takeConstants.secondaryOuttakeSpeed;
    }
  }
}
