package frc.robot.subsystems.shooter;

import java.util.Map.Entry;
import java.util.List;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;

public class Shooter extends SubsystemBase {
  
  private final TalonFX shootAngle = new TalonFX(Constants.IDs.shootangle);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final CANSparkFlex shooterLeft = new CANSparkFlex(Constants.IDs.shooterleft, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex shooterRight = new CANSparkFlex(Constants.IDs.shooterright, CANSparkLowLevel.MotorType.kBrushless);
  public final TimeOfFlight shooter_sensor = new TimeOfFlight(Constants.IDs.shootersensor);
  private RelativeEncoder m_encoder = shooterLeft.getEncoder();

   private SparkPIDController m_pidController = shooterLeft.getPIDController();
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private PolynomialSplineFunction m_shootAngleCurve;
  private PolynomialSplineFunction m_shooterCurve;
  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;

  public Shooter() {
    shooterLeft.restoreFactoryDefaults();
    shooterRight.restoreFactoryDefaults();
    shooterRight.follow(shooterLeft);
    shooterRight.burnFlash();
  

    // PID coefficients
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    shootAngle.setNeutralMode(NeutralModeValue.Brake);

    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 20;
    mm.MotionMagicAcceleration = 20;
    mm.MotionMagicJerk = 50;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 11;
    slot0.kI = 0.1;
    slot0.kD = 0.1;
    slot0.kV = 0.12;
    slot0.kS = 0.25;

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = shootAngle.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
    MAX_SHOOTING_DISTANCE = Constants.SHOOTER_MAP.get(Constants.SHOOTER_MAP.size() - 1).getKey();
    initializeShooterCurves(Constants.SHOOTER_MAP);
  }

  public static class State {
    public final double speed;
    public final double angle;

    public State(double shooterSpeed, double angle) {
      this.speed = shooterSpeed;
      this.angle = angle;
    }
  }

   private void initializeShooterCurves(List<Entry<Measure<Distance>, State>> shooterMap) {
    double[] distances = new double[shooterMap.size()];
    double[] shooterSpeeds = new double[shooterMap.size()];
    double[] angles = new double[shooterMap.size()];

    for (int i = 0; i < shooterMap.size(); i++) {
      distances[i] = shooterMap.get(i).getKey().in(Units.Meters);
      shooterSpeeds[i] = shooterMap.get(i).getValue().speed;
      angles[i] = shooterMap.get(i).getValue().angle;
    }

    m_shooterCurve = SPLINE_INTERPOLATOR.interpolate(distances, shooterSpeeds);
    m_shootAngleCurve = SPLINE_INTERPOLATOR.interpolate(distances, angles);
  }

  public State getAutomaticState(Vision vision) {
  var targetDistance = getTargetDistance(vision);
  var shooterSpeed = m_shooterCurve.value(targetDistance.in(Units.Meters));
  var angle = m_shootAngleCurve.value(targetDistance.in(Units.Meters));

  return new State(shooterSpeed, angle);
  }

  private Measure<Distance> getTargetDistance(Vision vision) {
  return Units.Meters.of(
  MathUtil.clamp(
  vision.getDistance(),
  MIN_SHOOTING_DISTANCE.in(Units.Meters),
  MAX_SHOOTING_DISTANCE.in(Units.Meters)
  )
    );
  }

  public void shooterOn(double setPointRotationsPerMinute) {
    m_pidController.setReference(setPointRotationsPerMinute, CANSparkMax.ControlType.kVelocity);
  }

  public void intakeHP() {
    shooterLeft.set(-0.2);
  }

  public void disableShooter() {
    shooterLeft.set(0);
  }

  public void lowerShootAngle(double angle) {
    shootAngle.setControl(m_mmReq.withPosition(angle).withSlot(0));
  }

    public void interpolatedShooterVelocity(State state) {
      m_pidController.setReference(state.speed, CANSparkMax.ControlType.kVelocity);
  }

  public void interpolatedShootAngle(State state) {
    shootAngle.setControl(m_mmReq.withPosition(state.angle).withSlot(0));
  }

  public void stowShootAngle() {
    shootAngle.setControl(m_mmReq.withPosition(0).withSlot(0));
  }

  public void zeroShootAngle() {
    shootAngle.setPosition(0);
  }

  public double shooterSensor() {
    return shooter_sensor.getRange();
  }

  public boolean shooterSensorOut() {
    return (shooter_sensor.getRange() < 300);
  }

  public boolean invShooterSensorOut() {
    return !(shooter_sensor.getRange() < 300);
  }

  public boolean isVelocitySet() {
    return((m_encoder.getVelocity() >= (2500-100)) && (m_encoder.getVelocity() <= (2500+100)));
  }

  public boolean isAngleSet() {
    return (m_mmReq.Position <= (shootAngle.getPosition().getValueAsDouble()+0.005) && m_mmReq.Position >= (shootAngle.getPosition().getValueAsDouble()-0.005));
  }

  public boolean isShooterSet() {
    return false;
  }

  public void periodic() {
    SmartDashboard.putNumber("Shoot Angle Position: ", shootAngle.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shoot Angle Velocity: ", shootAngle.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shoot Angle Power:", shootAngle.get());
    SmartDashboard.putNumber("Shoot Angle Voltage:", shootAngle.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Velocity: ", m_encoder.getVelocity());
    SmartDashboard.putNumber("Shooter Power:", shooterLeft.get());
    SmartDashboard.putBoolean("Shooter Velocity Setpoint Reached: ", isVelocitySet());
    SmartDashboard.putBoolean("Shooter Angle Setpoint Reached: ", isAngleSet());
    SmartDashboard.putBoolean("Shooter Setpoints Reached: ", isShooterSet());
  }

}
