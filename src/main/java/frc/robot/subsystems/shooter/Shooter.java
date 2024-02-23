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
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;

public class Shooter extends SubsystemBase {
  
  private final TalonFX shootAngle = new TalonFX(16);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final CANSparkFlex shooterLeft = new CANSparkFlex(19, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex shooterRight = new CANSparkFlex(20, CANSparkLowLevel.MotorType.kBrushless);
  public final TimeOfFlight shooter_sensor = new TimeOfFlight(2);
  private RelativeEncoder m_leftencoder = shooterLeft.getEncoder();

  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private PolynomialSplineFunction m_shootAngleCurve;
  private PolynomialSplineFunction m_shooterCurve;
  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;

  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(0.05, 12 / 6000);

  private final PIDController m_leftShooterFeedback = new PIDController(0.09, 0.0, 0.0);

  public Shooter() {
    shooterLeft.restoreFactoryDefaults();
    shooterRight.restoreFactoryDefaults();
    shooterRight.follow(shooterLeft);
    shooterRight.burnFlash();
    m_leftShooterFeedback.setTolerance(25);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    shootAngle.setNeutralMode(NeutralModeValue.Brake);

    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 10;
    mm.MotionMagicAcceleration = 10;
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

    public State(double speed, double angle) {
      this.speed = speed;
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

  private State getAutomaticState(Vision vision) {
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

  public void shooterOn(double setpointRotationsPerSecond) { //TODO: GATHER DATA FOR INTERPOLATE 1678 METHOD
    shooterLeft.set(
        m_shooterFeedforward.calculate(setpointRotationsPerSecond)
            + m_leftShooterFeedback.calculate(
                m_leftencoder.getVelocity() / 60, setpointRotationsPerSecond));
  }

  public void intakeHP() {
    shooterLeft.set(-0.4);
  }

  public void disableShooter() {
    shooterLeft.set(0);
  }

  public void lowerShootAngle() {
    shootAngle.setControl(m_mmReq.withPosition(0.18).withSlot(0)); // TODO: TUNE FOR INTERPOLATE 1678 METHOD
  }

    public void interpolatedShooterOn(State state) {
    shooterLeft.set(
        m_shooterFeedforward.calculate(state.speed)
            + m_leftShooterFeedback.calculate(
                m_leftencoder.getVelocity() / 60, state.speed));
  }

  public void interpolatedShootAngle(State state) {
    shootAngle.setControl(m_mmReq.withPosition(state.angle).withSlot(0));
  }

  public Command interpolatedShootAngleCommand(Vision vision) {
    return runEnd(
      () -> {interpolatedShootAngle(getAutomaticState(vision));},
      () -> {stowShootAngle();});

  }

  public Command interpolatedShooterOnCommand(Vision vision) {
    return runEnd(
      () -> {interpolatedShooterOn(getAutomaticState(vision));},
      () -> {disableShooter();});
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

  public boolean isShooterSet() {
    return (m_leftShooterFeedback.atSetpoint());
  }

  public boolean isAngleSet(Vision vision) {
    return (shootAngle.getPosition().getValueAsDouble()==getAutomaticState(vision).angle);
  }

  public void periodic() {
    SmartDashboard.putNumber("Shoot Angle Position: ", shootAngle.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shoot Angle Velocity: ", shootAngle.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shoot Angle Power:", shootAngle.get());
    SmartDashboard.putNumber("Shoot Angle Voltage:", shootAngle.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Velocity: ", m_leftencoder.getVelocity());
    SmartDashboard.putNumber("Shooter Power:", shooterLeft.get());
    SmartDashboard.putBoolean("Shooter Velocity Setpoint Reached: ", m_leftShooterFeedback.atSetpoint());
  }

}
