// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SubSystems.Intake;
import frc.robot.SubSystems.Outtake;
import frc.robot.SubSystems.Drivetrain2;
import frc.robot.SubSystems.Limelight;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.takeConstants;
import frc.robot.SubSystems.Arm;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private CommandXboxController controller;
  private CommandXboxController operator;
  private Intake m_intake;
  private Outtake m_Outtake;
  private Drivetrain2 drive;
  private Limelight limelight;
  private Arm m_arm;
  double limelightFwdDesire;
  double limelightRotDesire;
  double throttleValue = 0.5;
  double rotThrottle = 0.5;
  double leftPos;
  double rightPos;
  double desiredRot;
  double desiredLeftPos;
  double desiredRightPos;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    controller = new CommandXboxController(OperatorConstants.DRIVER_CONT_ID);
    operator = new CommandXboxController(OperatorConstants.OPERATOR_CONT_ID);
    m_intake = new Intake();
    m_Outtake = new Outtake();
    drive = new Drivetrain2();
    limelight = new Limelight();
    m_arm = new Arm();
    limelightFwdDesire = 0;
    limelightRotDesire = 0;

    // Intake controls

    operator.leftBumper().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> m_intake.Run(0), m_intake),
            Commands.runOnce(() -> m_Outtake.Run(0), m_Outtake),
            Commands.print("[I/O] INTAKE ENGAGED"),
            Commands.runOnce(() -> m_intake.Run(1), m_intake),
            Commands.waitUntil(m_intake::isNoteTouching).withTimeout(7),
            Commands.waitSeconds(takeConstants.intakeDelay),
            Commands.runOnce(() -> m_intake.Run(0), m_intake)));

    // Outtake speed changer controls

    operator.povDown().onTrue(Commands.sequence(Commands.runOnce(() -> m_Outtake.setSpeed(false), m_Outtake),
        Commands.print("[TD] Outtake throttle decreased")));
    operator.povDown().onFalse(Commands.sequence(Commands.runOnce(() -> m_Outtake.setSpeed(true), m_Outtake),
        Commands.print("[TI] Outtake throttle renormalized")));

    // "spit" the note out the back
  
    operator.x().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> m_intake.Run(0), m_intake),
            Commands.runOnce(() -> m_Outtake.Run(0), m_Outtake),
            Commands.print("[I/O] SPITTING NOTE"),
            Commands.runOnce(() -> m_intake.Run(-takeConstants.spitSpeed), m_intake),
            Commands.waitUntil(m_Outtake::isNoteNotTouching).withTimeout(5),
            Commands.waitSeconds(takeConstants.spitDelay),
            Commands.runOnce(() -> m_intake.Run(0), m_intake),
            Commands.runOnce(() -> m_Outtake.Run(0), m_Outtake)));

    // Outtake main controls

    operator.rightBumper().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> m_intake.Run(-0.2), m_intake),
            Commands.runOnce(() -> m_Outtake.Run(-0.2), m_Outtake),
            Commands.waitUntil(m_Outtake::isNoteNotTouching).withTimeout(0.5),
            Commands.runOnce(() -> m_intake.Run(0), m_intake),
            Commands.print("[I/O] OUTTAKE FIRING: Pew Pew!!"),
            Commands.runOnce(() -> m_Outtake.Run(m_Outtake.shootSpeed), m_Outtake),
            // Commands.runOnce(() -> m_intake.Run(1), m_intake),
            Commands.waitSeconds(takeConstants.outtakeShootDelay),
            Commands.runOnce(() -> m_intake.Run(1), m_intake),
            // Commands.waitUntil(m_Outtake::isNoteNotTouching).withTimeout(7),
            Commands.waitSeconds(takeConstants.outtakeDelay),
            Commands.runOnce(() -> m_Outtake.printRPM(), m_Outtake),
            Commands.runOnce(() -> m_intake.Run(0), m_intake),
            Commands.runOnce(() -> m_Outtake.Run(0), m_Outtake)));

    controller.povUp().onTrue(Commands.sequence(
        Commands.runOnce(() -> throttleValue += takeConstants.throttleIncrement),
        Commands.runOnce(() -> rotThrottle += takeConstants.throttleIncrement),
        Commands.print("[FSPEED] THROTTLE: " + throttleValue)));

    controller.povDown().onTrue(Commands.sequence(
        Commands.runOnce(() -> throttleValue -= takeConstants.throttleIncrement),
        Commands.runOnce(() -> rotThrottle -= takeConstants.throttleIncrement),
        Commands.print("[FSPEED] THROTTLE: " + throttleValue)));

    controller.rightBumper().whileTrue(Commands.run(() -> throttleValue = -throttleValue));
    controller.rightBumper().whileFalse(Commands.run(() -> throttleValue = Math.abs(throttleValue)));
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    limelightFwdDesire = 0;
    limelightRotDesire = 0;
  }

  @Override
  public void autonomousPeriodic() {
    if (limelight.hasValidTarget) {
      limelightFwdDesire = limelight.CalcFwdDesire(limelight.tz);
      limelightRotDesire = limelight.CalcRotDesire(limelight.tx);
      /*
       * Experimental automatic outtake system for the limelight
       * 
       * Just goes up to the limelight's "target", aligns with Amp, and fires outtake.
       * You will need to edit this for comp
       * 
       */
      if (limelightFwdDesire == 0 && limelightRotDesire == 0) {
        Commands.sequence(
          Commands.runOnce(() -> drive.arcadeDrive(0.5, 0)),
          Commands.waitUntil(m_arm::proximitySensor).withTimeout(10),
          Commands.runOnce(() -> drive.arcadeDrive(0, 0)),
          Commands.runOnce(() -> m_arm.Run(-1), m_arm),
          Commands.waitUntil(m_arm::armMaxSensor).withTimeout(5),
          Commands.runOnce(() -> m_intake.Run(0), m_intake),
          Commands.runOnce(() -> m_Outtake.Run(0), m_Outtake),
          Commands.runOnce(() -> m_arm.alignCustom(90), m_arm),
          Commands.runOnce(() -> m_intake.Run(-0.2), m_intake),
          Commands.runOnce(() -> m_Outtake.Run(-0.2), m_Outtake),
          Commands.waitUntil(m_Outtake::isNoteNotTouching).withTimeout(0.5),
          Commands.runOnce(() -> m_intake.Run(0), m_intake),
          Commands.print("[I/O] OUTTAKE FIRING: Pew Pew!!"),
          Commands.runOnce(() -> m_Outtake.Run(m_Outtake.shootSpeed), m_Outtake),
          // Commands.runOnce(() -> m_intake.Run(1), m_intake),
          Commands.waitSeconds(takeConstants.outtakeShootDelay),
          Commands.runOnce(() -> m_intake.Run(1), m_intake),
          // Commands.waitUntil(m_Outtake::isNoteNotTouching).withTimeout(7),
          Commands.waitSeconds(takeConstants.outtakeDelay),
          Commands.runOnce(() -> m_Outtake.printRPM(), m_Outtake),
          Commands.runOnce(() -> m_intake.Run(0), m_intake),
          Commands.runOnce(() -> m_Outtake.Run(0), m_Outtake)
        );
      } else {
      drive.arcadeDrive(limelightFwdDesire, limelightRotDesire);}
    } else {
      limelightRotDesire = limelight.CalcRotDesire(limelight.lastKnownTx);
      drive.arcadeDrive(0, limelightRotDesire);
    }
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    System.out.println("[INIT] ROBOT READY");
  }

  private double curveSensitivity(double val) {
    return val * Math.abs(val);
  }

  @Override
  public void teleopPeriodic() {
    throttleValue = MathUtil.clamp(throttleValue, -1, 1);
    rotThrottle = MathUtil.clamp(rotThrottle, -1, 1);
    drive.arcadeDrive((curveSensitivity(controller.getLeftY()) * throttleValue),
        (-controller.getRightX() * rotThrottle));
    m_arm.Run(0 - operator.getLeftTriggerAxis() + operator.getRightTriggerAxis());
  }

  @Override
  public void teleopExit() {
    System.out.println("[INF] DISENGAGING TELEOP");
  }

  // Encoder tests: drive 5 meters autonomously
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    leftPos = drive.getLeftPos().getValueAsDouble();
    rightPos = drive.getRightPos().getValueAsDouble();
    desiredRot = drive.metersToRotations(5);
    desiredLeftPos = desiredRot + leftPos;
    desiredRightPos = desiredRot + rightPos;

    drive.encoderDrive(desiredRot, desiredRot);
  }

  @Override
  public void testPeriodic() {
    leftPos = drive.getLeftPos().getValueAsDouble();
    rightPos = drive.getRightPos().getValueAsDouble();

    // double leftGo;
    // double rightGo;
    // if (leftPos <= desiredLeftPos) {
    //   leftGo = 0.3;
    // } else {
    //   leftGo = 0;
    // }
    // if (rightPos <= desiredRightPos) {
    //   rightGo = 0.3;
    // } else {
    //   rightGo = 0;
    // }
    // drive.tankDrive(leftGo, rightGo);

    System.out.println("Right: " + rightPos + "Left: " + leftPos);
  }

  @Override
  public void testExit() {
  }
}
