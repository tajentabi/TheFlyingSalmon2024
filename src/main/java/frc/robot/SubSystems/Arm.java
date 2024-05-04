package frc.robot.SubSystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.armConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Arm extends SubsystemBase {
    private final CANSparkMax m_left = new CANSparkMax(armConstants.ARM_ID_L, MotorType.kBrushless);
    private final CANSparkMax m_right = new CANSparkMax(armConstants.ARM_ID_R, MotorType.kBrushless);
    private final RelativeEncoder encoderR = m_right.getEncoder();
    private final DigitalInput sensorLeft = new DigitalInput(armConstants.SENSOR_L_ID);
    private final DigitalInput sensorRight = new DigitalInput(armConstants.SENSOR_R_ID);
    private final DigitalInput proxSensor = new DigitalInput(armConstants.PROX_SENSOR_ID);
    private final SparkPIDController pidR = m_right.getPIDController();
    private final SparkPIDController pidL = m_left.getPIDController();
    private double rotations;
    private double degrees;

    public Arm() {
        m_left.restoreFactoryDefaults();
        m_right.restoreFactoryDefaults();
        m_right.setSmartCurrentLimit(35);
        m_left.setSmartCurrentLimit(35);
        m_left.enableVoltageCompensation(armConstants.VOLTAGE_MAX);
        m_right.enableVoltageCompensation(armConstants.VOLTAGE_MAX);
        m_left.clearFaults();
        m_right.clearFaults();
        m_left.setInverted(true);
        m_right.setClosedLoopRampRate(1);
        m_right.setOpenLoopRampRate(1);
        m_left.setClosedLoopRampRate(1);
        m_left.setOpenLoopRampRate(1);
        m_left.setIdleMode(IdleMode.kBrake);
        m_right.setIdleMode(IdleMode.kBrake);
        pidR.setFeedbackDevice(encoderR);
        pidR.setP(armConstants.ARM_ENCODER_PROPORTIONALITY_GAIN);
        pidR.setD(armConstants.ARM_ENCODER_DERIVATIVE_GAIN);
        pidR.setI(armConstants.ARM_ENCODER_INTEGRAL_GAIN);
        pidR.setOutputRange(-0.5, 0.5);
        pidL.setFeedbackDevice(encoderR);
        pidL.setP(armConstants.ARM_ENCODER_PROPORTIONALITY_GAIN);
        pidL.setD(armConstants.ARM_ENCODER_DERIVATIVE_GAIN);
        pidL.setI(armConstants.ARM_ENCODER_INTEGRAL_GAIN);
        pidL.setOutputRange(-0.5, 0.5);
        m_right.burnFlash();
        m_left.burnFlash();
    }

    public boolean armMaxSensor() {
        if (sensorLeft.get() || sensorRight.get()) {
            return true;
        } else {
            return false;
        }
    }

    public void Run(double speed) {
        speed *= armConstants.VOLTAGE_MAX;
        if (sensorRight.get() || sensorLeft.get() && speed < 0) {
            m_right.setVoltage(0);
            m_left.setVoltage(0);
        } else {
            m_right.setVoltage(speed);
            m_left.setVoltage(speed);
        }
    }

    public boolean proximitySensor() {
        if (!proxSensor.get()) {
            return true;
        } else {
            return false;
        }
    }

    // This might also work (but idk test the uncommented version first)

    // public void encoderRun(double angle) {
    //     rotations = (armConstants.ARM_RATIO / 360) * angle;
    //     encoderL.setPosition(rotations);
    //     encoderR.setPosition(rotations);
    // }

    public void alignCustom(double angle) {
        rotations = (armConstants.ARM_RATIO / 360) * angle;
        pidR.setReference(rotations, CANSparkMax.ControlType.kPosition);
        pidL.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    public void alignAmp() {
        // 0.6604 (meters for height of Amp) divide this by stopping dist if it is not 1
        degrees = Math.tan(0.6604 / LimelightConstants.desiredStoppingDist);
        degrees += armConstants.ANGLE_OFFSET_AMP;
        rotations = (armConstants.ARM_RATIO / 360) * degrees;
        pidR.setReference(rotations, CANSparkMax.ControlType.kPosition);
        pidL.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    public void alignSpeaker() {
        // 2.05 (meters for height of speaker) divide this by stopping dist if it is not 1
        degrees = Math.tan(2.05 / LimelightConstants.desiredStoppingDist);
        degrees += armConstants.ANGLE_OFFSET_SPEAKER;
        rotations = (armConstants.ARM_RATIO / 360) * degrees;
        pidR.setReference(rotations, CANSparkMax.ControlType.kPosition);
        pidL.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }
}
