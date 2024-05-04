// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Modified from https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/CommandBasedDrive

package frc.robot.SubSystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.TalonFXConstants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public class Drivetrain2 extends SubsystemBase {
  private final TalonFX m_leftLeader = new TalonFX(LEFT_LEADER_ID, CANBUS_NAME);
  private final TalonFX m_leftFollower = new TalonFX(LEFT_FOLLOWER_ID, CANBUS_NAME);
  private final TalonFX m_rightLeader = new TalonFX(RIGHT_LEADER_ID, CANBUS_NAME);
  private final TalonFX m_rightFollower = new TalonFX(RIGHT_FOLLOWER_ID, CANBUS_NAME);
  private final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
  private final DutyCycleOut m_leftOut = new DutyCycleOut(0); // Initialize with 0% output
  private final DutyCycleOut m_rightOut = new DutyCycleOut(0); // Initialize with 0% output
  /*
   * Slew Rate Limiters
   * Prevents a change larger than limit/second
   */
  private final SlewRateLimiter fwdFilter = new SlewRateLimiter(SLEW_RATE_LIMIT);
  private final SlewRateLimiter rotFilter = new SlewRateLimiter(SLEW_RATE_LIMIT);
  private final SlewRateLimiter leftFilter = new SlewRateLimiter(SLEW_RATE_LIMIT);
  private final SlewRateLimiter rightFilter = new SlewRateLimiter(SLEW_RATE_LIMIT);

  /*
   * These numbers are an example AndyMark Drivetrain with some additional weight.
   * This is a fairly light robot.
   * Note you can utilize results from robot characterization instead of
   * theoretical numbers.
   * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
   */
  private final double kGearRatio = GEAR_RATIO;
  private final double kWheelRadiusInches = WHEEL_RADIUS_INCHES;

  private double leftRotations;
  private double rightRotations;

  /** Creates a new Drivetrain. */
  public Drivetrain2() {

    initializeLeftDriveTalonFX(m_leftLeader.getConfigurator());
    initializeLeftDriveTalonFX(m_leftFollower.getConfigurator());
    initializeRightDriveTalonFX(m_rightLeader.getConfigurator());
    initializeRightDriveTalonFX(m_rightFollower.getConfigurator());

    /* Set followers to follow leader */
    m_leftFollower.setControl(new Follower(m_leftLeader.getDeviceID(), false));
    m_rightFollower.setControl(new Follower(m_rightLeader.getDeviceID(), false));

    m_leftLeader.setSafetyEnabled(true);
    m_rightLeader.setSafetyEnabled(true);

    /* Make sure all critical signals are synchronized */
    /*
     * Setting all these signals to 100hz means they get sent at the same time if
     * they're all on a CANivore
     * 50hz if on CAN 2.0
     */
    m_leftLeader.getPosition().setUpdateFrequency(50);
    m_rightLeader.getPosition().setUpdateFrequency(50);

    /*
     * Set the update frequency of the main requests to 0 so updates are sent
     * immediately in the arcadeDrive method
     */
    m_leftOut.UpdateFreqHz = 0;
    m_rightOut.UpdateFreqHz = 0;
  }

  /**
   * Drive the robot using an arcade drive format.
   * 
   * This must be called periodically or else the control frames will not
   * get sent out, resulting in the TalonFXs disabling
   * 
   * @param fwd Forward/Reverse output
   * @param rot Left/Right output
   */
  public void arcadeDrive(double fwd, double rot) {
    fwd = fwdFilter.calculate(fwd);
    rot = rotFilter.calculate(rot);
    m_leftOut.Output = MathUtil.clamp((fwd + rot), -1, 1);
    m_rightOut.Output = MathUtil.clamp((fwd - rot), -1, 1);

    m_leftLeader.setControl(m_leftOut);
    m_rightLeader.setControl(m_rightOut);
  }

  public void encoderDrive(double leftPos, double rightPos) {
    leftRotations = metersToRotations(leftPos);
    rightRotations = metersToRotations(rightPos);
    m_leftLeader.setControl(m_request.withPosition(leftRotations));
    m_rightLeader.setControl(m_request.withPosition(rightRotations));
  }

  /**
   * Drive the robot using a tank drive format.
   * 
   * This must be called periodically or else the control frames will not
   * get sent out, resulting in the TalonFXs disabling
   * 
   * @param left  Forward/Reverse output
   * @param right Left/Right output
   */

  public void tankDrive(double left, double right) {
    left = leftFilter.calculate(left);
    right = rightFilter.calculate(right);
    left = MathUtil.clamp(left, -1, 1);
    right = MathUtil.clamp(right, -1, 1);
    m_leftOut.Output = left;
    m_rightOut.Output = right;

    m_leftLeader.setControl(m_leftOut);
    m_rightLeader.setControl(m_rightOut);
  }

  /*
   * Sets the Drivetrain to neutral
   */
  public void stop() {
    m_leftOut.Output = 0;
    m_rightOut.Output = 0;

    m_leftLeader.setControl(m_leftOut);
    m_rightLeader.setControl(m_rightOut);
  }

  public StatusSignal<Double> getLeftPos() {
    return m_leftLeader.getPosition();
  }

  public StatusSignal<Double> getRightPos() {
    return m_rightLeader.getPosition();
  }

  public double rotationsToMeters(double rotations) {
    /* Get circumference of wheel */
    final double circumference = this.kWheelRadiusInches * 2 * Math.PI;
    /* Every rotation of the wheel travels this many inches */
    /* So now get the meters traveled per rotation */
    final double metersPerWheelRotation = Units.inchesToMeters(circumference);
    /* Now apply gear ratio to input rotations */
    double gearedRotations = rotations / this.kGearRatio;
    /* And multiply geared rotations by meters per rotation */
    return gearedRotations * metersPerWheelRotation;
  }

  public double metersToRotations(double meters) {
    /* Get circumference of wheel */
    final double circumference = this.kWheelRadiusInches * 2 * Math.PI;
    /* Every rotation of the wheel travels this many inches */
    /* So now get the rotations per meter traveled */
    final double wheelRotationsPerMeter = 1.0 / Units.inchesToMeters(circumference);
    /* Now apply wheel rotations to input meters */
    double wheelRotations = wheelRotationsPerMeter * meters;
    /* And multiply by gear ratio to get rotor rotations */
    return wheelRotations * this.kGearRatio;
  }

  /**
   * Initialize a left drive TalonFX device from the configurator object
   * 
   * @param cfg Configurator of the TalonFX device
   */
  private void initializeLeftDriveTalonFX(TalonFXConfigurator cfg) {
    var toApply = new TalonFXConfiguration();

    var slot0Configs = toApply.Slot0;
    slot0Configs.kS = K_S; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = K_V; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = K_A; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = K_P; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = K_I; // no output for integrated error
    slot0Configs.kD = K_D; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = toApply.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = MAGIC_CRUISE_VELOCITY; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicExpo_kV = MAGIC_EXPO_K_V; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicExpo_kA = MAGIC_EXPO_K_A;

    /*
     * User can change configs if they want, or leave this blank for factory-default
     */
    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    cfg.apply(toApply);

    /* And initialize position to 0 */
    cfg.setPosition(0);
  }

  /**
   * Initialize a right drive TalonFX device from the configurator object
   * 
   * @param cfg Configurator of the TalonFX device
   */
  private void initializeRightDriveTalonFX(TalonFXConfigurator cfg) {
    var toApply = new TalonFXConfiguration();

    var slot0Configs = toApply.Slot0;
    slot0Configs.kS = K_S; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = K_V; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = K_A; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = K_P; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = K_I; // no output for integrated error
    slot0Configs.kD = K_D; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = toApply.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = MAGIC_CRUISE_VELOCITY; 
    motionMagicConfigs.MotionMagicExpo_kV = MAGIC_EXPO_K_V; 
    motionMagicConfigs.MotionMagicExpo_kA = MAGIC_EXPO_K_A;

    /*
     * User can change configs if they want, or leave this blank for factory-default
     */
    toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    cfg.apply(toApply);

    /* And initialize position to 0 */
    cfg.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
