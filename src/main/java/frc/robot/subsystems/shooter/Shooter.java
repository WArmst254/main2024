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
  
  private final TalonFX shooter = new TalonFX(Constants.IDs.shooter);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final CANSparkFlex flywheelLeft = new CANSparkFlex(Constants.IDs.flywheelLeft, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex flywheelRight = new CANSparkFlex(Constants.IDs.flywheelRight, CANSparkLowLevel.MotorType.kBrushless);
  public final TimeOfFlight shooter_sensor = new TimeOfFlight(Constants.IDs.shootersensor);
  private RelativeEncoder m_encoder = flywheelLeft.getEncoder();

  private SparkPIDController m_pidController = flywheelLeft.getPIDController();
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private PolynomialSplineFunction m_shooterCurve;
  private PolynomialSplineFunction m_flywheelCurve;
  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;

  public Shooter() {

    /*Flywheel Settings and Gains */
    flywheelLeft.restoreFactoryDefaults();
    flywheelRight.restoreFactoryDefaults();
    flywheelRight.follow(flywheelLeft);
    flywheelRight.burnFlash();
  

    // PID coefficients for Flywheels
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /*Shooter Angle Settings and Motion Magic Constants */

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    shooter.setNeutralMode(NeutralModeValue.Brake);

    /* Motion Profiling Constants For Shooter Angle*/
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 20;
    mm.MotionMagicAcceleration = 20;
    mm.MotionMagicJerk = 50;

    // PID Coefficients for Shooter Angle
    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 20;
    slot0.kI = 10;
    slot0.kD = 0.1;
    slot0.kV = 0.12;
    slot0.kS = 0.25;

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = shooter.getConfigurator().apply(cfg);
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

    public State(double speed, double angle) {
      this.speed = speed;
      this.angle = angle;
    }
  }

   private void initializeShooterCurves(List<Entry<Measure<Distance>, State>> shooterMap) {
    double[] distances = new double[shooterMap.size()];
    double[] speeds = new double[shooterMap.size()];
    double[] angles = new double[shooterMap.size()];

    for (int i = 0; i < shooterMap.size(); i++) {
      distances[i] = shooterMap.get(i).getKey().in(Units.Meters);
      speeds[i] = shooterMap.get(i).getValue().speed;
      angles[i] = shooterMap.get(i).getValue().angle;
    }

    m_shooterCurve = SPLINE_INTERPOLATOR.interpolate(distances, angles);
    m_flywheelCurve = SPLINE_INTERPOLATOR.interpolate(distances, speeds);
  }

  public State getAutomaticState(Vision vision) {
    var targetDistance = getTargetDistance(vision);
    var speed = m_flywheelCurve.value(targetDistance.in(Units.Meters));
    var angle = m_shooterCurve.value(targetDistance.in(Units.Meters));

    return new State(speed, angle);
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

  public void flywheelsOn(double setPointRotationsPerMinute) {
    m_pidController.setReference(setPointRotationsPerMinute, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("Flywheel Taget Velocity", setPointRotationsPerMinute);
  }

  public void intakeHP() {
    flywheelLeft.set(-0.2);
  }

  public void disableFlywheels() {
    flywheelLeft.set(0);
  }

  public void lowerShooter(double angle) {
    shooter.setControl(m_mmReq.withPosition(angle).withSlot(0));
  }

  public void interpolatedFlywheelVelocity(State state) {
    m_pidController.setReference(state.speed, CANSparkMax.ControlType.kVelocity);
  }

  public void interpolatedShooterAngle(State state) {
    shooter.setControl(m_mmReq.withPosition(state.angle).withSlot(0));
  }

  public void stowShooter() {
    shooter.setControl(m_mmReq.withPosition(0).withSlot(0));
  }

  public void zeroShooter() {
    shooter.setPosition(0);
  }

  public double shooterSensor() {
    return shooter_sensor.getRange();
  }

  public boolean shooterSensorOut() {
    return (shooter_sensor.getRange() < 335);
  }

  public boolean invShooterSensorOut() {
    return !(shooter_sensor.getRange() < 335);
  }

  public boolean isVelocitySet() {
    return((m_encoder.getVelocity() >= 2500-100) && (m_encoder.getVelocity() <= (2500+100)));
  }

  public boolean isAngleSet() {
    return (m_mmReq.Position <= (shooter.getPosition().getValueAsDouble()+0.01) && m_mmReq.Position >= (shooter.getPosition().getValueAsDouble()-0.01));
  }

  public boolean isShooterSet() {
    return isAngleSet() && isAngleSet();
  }

  public void periodic() {
    SmartDashboard.putNumber("Shooter Angle Position: ", shooter.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Angle Velocity: ", shooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Angle Power:", shooter.get());
    SmartDashboard.putNumber("Shooter Angle Voltage:", shooter.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Flywheels Velocity: ", m_encoder.getVelocity());
    SmartDashboard.putNumber("Flywheels Power:", flywheelLeft.get());
    SmartDashboard.putBoolean("Flywheels Velocity Setpoint Reached: ", isVelocitySet());
    SmartDashboard.putBoolean("Shooter Angle Setpoint Reached: ", isAngleSet());
    SmartDashboard.putBoolean("Shooter Setpoints Reached: ", isShooterSet());
  }

}
