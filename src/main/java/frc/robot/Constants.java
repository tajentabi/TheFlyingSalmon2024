// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class DrivetrainConstants {
    public static final double GEAR_RATIO = 7.0455; // The Flying Salmon has a 7.0455 gearbox ratio
    public static final double WHEEL_RADIUS_INCHES = 2; // The Flying Salmon has 4" diameter wheels / 2
    public static final double SLEW_RATE_LIMIT = 3.5;
    public static final double K_S = 0.25;
    public static final double K_A = 0.01;
    public static final double K_V = 0.12;
    public static final double K_P = 4.8;
    public static final double K_I = 0;
    public static final double K_D = 0.1;
    public static final double MAGIC_CRUISE_VELOCITY = 80;
    public static final double MAGIC_EXPO_K_V = 0.12;
    public static final double MAGIC_EXPO_K_A = 0.1;
  }
  
  public static class TalonFXConstants {
    public static final String CANBUS_NAME = "rio";

    /* Talon FX Device IDs */
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;
  }

  public static class LimelightConstants {
    // Denominator for autonomous guidance system speed suppressor. Should be 1 during comp
    public static final int SPEED_SUPPRESSION_DENOM = 3;
    // Distance in meters automatic guidance should stop from a target (make sure to define that target for limelight)
    public static final double desiredStoppingDist = 0.01;
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONT_ID = 0;
    public static final int OPERATOR_CONT_ID = 1;
  }

  public static class armConstants {
    public static final int ARM_ID_R = 8;
    public static final int ARM_ID_L = 9;
    public static final int SENSOR_L_ID = 1;
    public static final int SENSOR_R_ID = 2;
    public static final int PROX_SENSOR_ID = 3;
    public static final double ARM_RATIO = 78.75;
    // Value for kP coefficient. Edit during testing to help with arm alignment
    public static final double ARM_ENCODER_PROPORTIONALITY_GAIN = 0.1;
    public static final double ARM_ENCODER_INTEGRAL_GAIN = 0;
    public static final double ARM_ENCODER_DERIVATIVE_GAIN = 0;
    // the final angle for the encoderRun is the calculated value plus ANGLE_OFFSET
    public static final double ANGLE_OFFSET_AMP = 5;
    public static final double ANGLE_OFFSET_SPEAKER = 5;
    // Voltage cap for arm motors
    public static final double VOLTAGE_MAX = 10;
  }

  public static class takeConstants {
    public static final int INTAKE_ID = 5;
    public static final int OUTAKE_LEADER_ID = 6;
    public static final int OUTAKE_FOLLOWER_ID = 7;
    // lower throttled outtake speed
    public static final double secondaryOuttakeSpeed = 0.33;
    // Delay between limit switch detection and motor shutoff (seconds)
    public static final double intakeDelay = 0.2;
    // Same thing but for outtake
    public static final double outtakeDelay = 1;
    // Speed of spit
    public static final double spitSpeed = 0.5;
    // Delay between limit switch signal loss and motor shutoff (seconds) for spit function
    public static final double spitDelay = 0.5;
    // Increment for drivetrain throttle
    public static final double throttleIncrement = 0.2;
    public static final double outtakeShootDelay = 0.5;
  }
}
