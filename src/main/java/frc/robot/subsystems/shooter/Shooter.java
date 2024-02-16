package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
  private final CANSparkFlex shooterLeft =
      new CANSparkFlex(19, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex shooterRight =
      new CANSparkFlex(20, CANSparkLowLevel.MotorType.kBrushless);
  private final TalonFX feedFront = new TalonFX(13);
  private final TalonFX feedBack = new TalonFX(14);
  private SparkPIDController m_leftpidController;
  private SparkPIDController m_rightpidController;

  private RelativeEncoder m_leftencoder;
  private RelativeEncoder m_rightencoder;
  public double left_kP,
      left_kI,
      left_kD,
      left_kIz,
      left_kFF,
      left_kMaxOutput,
      left_kMinOutput,
      left_maxRPM;
  public double right_kP,
      right_kI,
      right_kD,
      right_kIz,
      right_kFF,
      right_kMaxOutput,
      right_kMinOutput,
      right_maxRPM;

  public Shooter() {

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters in the
     * SPARK MAX to their factory default state. If no argument is passed, these parameters will not
     * persist between power cycles
     */
    shooterLeft.restoreFactoryDefaults();
    shooterRight.restoreFactoryDefaults();
    /**
     * In order to use PID functionality for a controller, a SparkPIDController object is
     * constructed by calling the getPIDController() method on an existing CANSparkMax object
     */
    m_leftpidController = shooterLeft.getPIDController();
    m_rightpidController = shooterRight.getPIDController();
    // Encoder object created to display position values
    m_leftencoder = shooterLeft.getEncoder();
    m_rightencoder = shooterRight.getEncoder();

    // PID coefficients
    left_kP = 6e-5;
    left_kI = 0;
    left_kD = 0;
    left_kIz = 0;
    left_kFF = 0.000015;
    left_kMaxOutput = 1;
    left_kMinOutput = -1;
    left_maxRPM = 6000;

    right_kP = 6e-5;
    right_kI = 0;
    right_kD = 0;
    right_kIz = 0;
    right_kFF = 0.000015;
    right_kMaxOutput = 1;
    right_kMinOutput = -1;
    right_maxRPM = 6000;

    // set PID coefficients
    m_leftpidController.setP(left_kP);
    m_leftpidController.setI(left_kI);
    m_leftpidController.setD(left_kD);
    m_leftpidController.setIZone(left_kIz);
    m_leftpidController.setFF(left_kFF);
    m_leftpidController.setOutputRange(left_kMinOutput, left_kMaxOutput);

    m_rightpidController.setP(right_kP);
    m_rightpidController.setI(right_kI);
    m_rightpidController.setD(right_kD);
    m_rightpidController.setIZone(right_kIz);
    m_rightpidController.setFF(right_kFF);
    m_rightpidController.setOutputRange(right_kMinOutput, right_kMaxOutput);
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("L P Gain", left_kP);
    SmartDashboard.putNumber("L I Gain", left_kI);
    SmartDashboard.putNumber("L D Gain", left_kD);
    SmartDashboard.putNumber("L I Zone", left_kIz);
    SmartDashboard.putNumber("L Feed Forward", left_kFF);
    SmartDashboard.putNumber("L Max Output", left_kMaxOutput);
    SmartDashboard.putNumber("L Min Output", left_kMinOutput);

    SmartDashboard.putNumber("R P Gain", right_kP);
    SmartDashboard.putNumber("R I Gain", right_kI);
    SmartDashboard.putNumber("R D Gain", right_kD);
    SmartDashboard.putNumber("R I Zone", right_kIz);
    SmartDashboard.putNumber("R Feed Forward", right_kFF);
    SmartDashboard.putNumber("R Max Output", right_kMaxOutput);
    SmartDashboard.putNumber("R Min Output", right_kMinOutput);

    SmartDashboard.putNumber("L ProcessVariable", m_leftencoder.getVelocity());
    SmartDashboard.putNumber("R ProcessVariable", m_rightencoder.getVelocity());
  }

  public void shooterAutoOnCommand() {
    // read PID coefficients from SmartDashboard
    double l_p = SmartDashboard.getNumber("L P Gain", 0);
    double l_i = SmartDashboard.getNumber("L I Gain", 0);
    double l_d = SmartDashboard.getNumber("L D Gain", 0);
    double l_iz = SmartDashboard.getNumber("L I Zone", 0);
    double l_ff = SmartDashboard.getNumber("L Feed Forward", 0);
    double l_max = SmartDashboard.getNumber("L Max Output", 0);
    double l_min = SmartDashboard.getNumber("L Min Output", 0);

    double r_p = SmartDashboard.getNumber("R P Gain", 0);
    double r_i = SmartDashboard.getNumber("R I Gain", 0);
    double r_d = SmartDashboard.getNumber("R D Gain", 0);
    double r_iz = SmartDashboard.getNumber("R I Zone", 0);
    double r_ff = SmartDashboard.getNumber("R Feed Forward", 0);
    double r_max = SmartDashboard.getNumber("R Max Output", 0);
    double r_min = SmartDashboard.getNumber("R Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((l_p != left_kP)) {
      m_leftpidController.setP(l_p);
      left_kP = l_p;
    }
    if ((l_i != left_kI)) {
      m_leftpidController.setI(l_i);
      left_kI = l_i;
    }
    if ((l_d != left_kD)) {
      m_leftpidController.setD(l_d);
      left_kD = l_d;
    }
    if ((l_iz != left_kIz)) {
      m_leftpidController.setIZone(l_iz);
      left_kIz = l_iz;
    }
    if ((l_ff != left_kFF)) {
      m_leftpidController.setFF(l_ff);
      left_kFF = l_ff;
    }
    if ((l_max != left_kMaxOutput) || (l_min != left_kMinOutput)) {
      m_leftpidController.setOutputRange(l_min, l_max);
      left_kMinOutput = l_min;
      left_kMaxOutput = l_max;
    }

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((r_p != right_kP)) {
      m_rightpidController.setP(r_p);
      right_kP = r_p;
    }
    if ((r_i != right_kI)) {
      m_rightpidController.setI(r_i);
      right_kI = r_i;
    }
    if ((r_d != right_kD)) {
      m_rightpidController.setD(r_d);
      right_kD = r_d;
    }
    if ((r_iz != right_kIz)) {
      m_rightpidController.setIZone(r_iz);
      right_kIz = r_iz;
    }
    if ((r_ff != right_kFF)) {
      m_rightpidController.setFF(r_ff);
      right_kFF = r_ff;
    }
    if ((r_max != right_kMaxOutput) || (r_min != right_kMinOutput)) {
      m_rightpidController.setOutputRange(r_min, r_max);
      right_kMinOutput = r_min;
      right_kMaxOutput = r_max;
    }

    /**
     * PIDController objects are commanded to a set point using the SetReference() method.
     *
     * <p>The first parameter is the value of the set point, whose units vary depending on the
     * control type set in the second parameter.
     *
     * <p>The second parameter is the control type can be set to one of four parameters:
     * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     * com.revrobotics.CANSparkMax.ControlType.kPosition
     * com.revrobotics.CANSparkMax.ControlType.kVelocity
     * com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    double r_setPoint = right_maxRPM;
    m_rightpidController.setReference(r_setPoint, CANSparkMax.ControlType.kVelocity);
    double l_setPoint = left_maxRPM;
    m_leftpidController.setReference(l_setPoint, CANSparkMax.ControlType.kVelocity);

    if (shooterLeft.getEncoder().getVelocity() >= (left_maxRPM - 300)
        && shooterRight.getEncoder().getVelocity() >= (right_maxRPM - 300)) {
      feedFront.set(-0.95);
      feedBack.set(0.95);
    }
  }

  public void periodic() {
    SmartDashboard.putNumber("L ProcessVariable", m_leftencoder.getVelocity());
    SmartDashboard.putNumber("R ProcessVariable", m_rightencoder.getVelocity());
    System.out.println();
  }
}
