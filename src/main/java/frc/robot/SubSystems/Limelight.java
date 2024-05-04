// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public LimelightHelpers.LimelightResults results;
  public double tx;
  public double ty;
  public double ta;
  public double tz;
  // Area
  // private double desiredArea;
  // private double desiredAreaSqrt;
  public boolean hasValidTarget;
  // Memory
  public double lastKnownTx = 0;
  public double lastKnownTy = 0;
  public double lastKnownTz = 0;

  //  Just in case ;)
  // public double getDesiredArea() {
  //   return desiredArea;
  // }

  // public double getDesiredAreaSqrt() {
  //   return desiredAreaSqrt;
  // }

  // public void setDesiredArea(double area) {
  //   area = MathUtil.clamp(area, 0, Double.POSITIVE_INFINITY);
  //   desiredArea = area;
  //   desiredAreaSqrt = Math.sqrt(area);
  // }

  // I like funny math curves
  public double CalcFwdDesire(double z) {
    // On desmos: y = ((x-1) * abs(x-1) * 2)
    double speed = ((z - LimelightConstants.desiredStoppingDist) * Math.abs(z - LimelightConstants.desiredStoppingDist)) * 2;
    speed = MathUtil.clamp(speed, -1, 1);
    return speed / LimelightConstants.SPEED_SUPPRESSION_DENOM;
  }

  public double CalcRotDesire(double x) {
    // More complicated non-linear algorithim. Could be useful?
    // double speed = (Math.atan(tx*0.1)) / 1.4;
    double speed = x / 15;
    speed = MathUtil.clamp(speed, -1, 1);
    return speed / LimelightConstants.SPEED_SUPPRESSION_DENOM;
  }

  public void UpdateLimelight() {
    // Might have to add AprilTag ID differentiation to this
    try {
      boolean tv = LimelightHelpers.getTV("");
      if (tv) {
        hasValidTarget = true;
        results = LimelightHelpers.getLatestResults("");
        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("limelight");
        NetworkTableEntry tzEntry = table.getEntry("ts");
        tz = tzEntry.getDouble(0.0);
      } else {
        hasValidTarget = false;
        lastKnownTx = tx;
        lastKnownTy = ty;
        lastKnownTz = tz;
      }
    } catch (Exception e) {
      tx = 0.001;
      ty = 0.001;
      tz = 0.001;
      hasValidTarget = false;
      System.out.println("[ERR:FATAL] Failed to read telemetry from Limelight: " + e);
    }
  }

  public Limelight() {
    UpdateLimelight();
  }

  @Override
  public void periodic() {
    UpdateLimelight();
  }
}
