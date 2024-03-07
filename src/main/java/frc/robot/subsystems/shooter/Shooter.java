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
import frc.robot.util.TunableNumber;

public class Shooter extends SubsystemBase {
  
  private final TalonFX shooter = new TalonFX(Constants.IDs.shooter);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  private final CANSparkFlex flywheelLeft = new CANSparkFlex(Constants.IDs.flywheelLeft, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex flywheelRight = new CANSparkFlex(Constants.IDs.flywheelRight, CANSparkLowLevel.MotorType.kBrushless);
  private RelativeEncoder m_encoder = flywheelLeft.getEncoder();

  public final TimeOfFlight shooter_sensor = new TimeOfFlight(Constants.IDs.shootersensor);

  private SparkPIDController m_pidController = flywheelLeft.getPIDController();
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  private TunableNumber fP = new TunableNumber("Shooter FlyWheel PID/P");
  private TunableNumber fI = new TunableNumber("Shooter FlyWheel PID/I");
  private TunableNumber fD = new TunableNumber("Shooter FlyWheel PID/D");
  private TunableNumber fIz = new TunableNumber("Shooter FlyWheel PID/IZ");
  private TunableNumber fFF = new TunableNumber("Shooter FlyWheel PID/FF");
  private TunableNumber fThreshold = new TunableNumber("Shooter Flywheel/Threshold RPM");

  private TunableNumber mm_sVelocity = new TunableNumber("Shooter Angle/Velocity");
  private TunableNumber mm_sAcceleration = new TunableNumber("Shooter Angle/Acceleration");
  private TunableNumber mm_sJerk = new TunableNumber("Shooter Angle/Jerk");
  private TunableNumber sP = new TunableNumber("Shooter Angle PID/P");
  private TunableNumber sI = new TunableNumber("Shooter Angle PID/I");
  private TunableNumber sD = new TunableNumber("Shooter Angle PID/D");
  private TunableNumber sV = new TunableNumber("Shooter Angle PID/V");
  private TunableNumber sS = new TunableNumber("Shooter Angle PID/S");
  private TunableNumber sThreshold = new TunableNumber("Shooter Angle/Threshold");

  private TunableNumber subRPM = new TunableNumber("Subwoofer Shot/RPM");
  private TunableNumber subAngle = new TunableNumber("SubwooferShot/Angle");

  private TunableNumber podRPM = new TunableNumber("Podium Shot/RPM");
  private TunableNumber podAngle = new TunableNumber("Podium Shot/Angle");

  private TunableNumber intakeHPspeed = new TunableNumber("HP Intake/Speed");

  private TunableNumber intakingAngle = new TunableNumber("Intake/Angle");

  private TunableNumber shooterSensorRange = new TunableNumber("Sensor/Shooter Sensor Range");



  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private PolynomialSplineFunction m_shooterCurve;
  private PolynomialSplineFunction m_flywheelCurve;
  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;

  public Shooter() {

    subRPM.setDefault(ShooterConstants.subwooferRPM);
    subAngle.setDefault(ShooterConstants.subwooferAngle);

    podRPM.setDefault(ShooterConstants.podiumRPM);
    podAngle.setDefault(ShooterConstants.podiumAngle);

    intakeHPspeed.setDefault(ShooterConstants.humanPlayerIntakeSpeed);
    intakingAngle.setDefault(ShooterConstants.groundIntakeAngle);

    shooterSensorRange.setDefault(ShooterConstants.TOFsensorRange);

    sThreshold.setDefault(ShooterConstants.angleThreshold);
    fThreshold.setDefault(ShooterConstants.flywheelThreshold);
    
    /*Flywheel Settings and Gains */
    flywheelLeft.restoreFactoryDefaults();
    flywheelRight.restoreFactoryDefaults();
    flywheelRight.follow(flywheelLeft);
    flywheelRight.burnFlash();
  
    fP.setDefault(ShooterConstants.flywheelP);
    fI.setDefault(ShooterConstants.flywheelI);
    fD.setDefault(ShooterConstants.flywheelD);
    fIz.setDefault(ShooterConstants.flywheelIZ);
    fFF.setDefault(ShooterConstants.flywheelFF);
    
    // PID coefficients for Flywheels
    kP = fP.get();
    kI = fI.get();
    kD = fD.get();
    kIz = fIz.get(); 
    kFF = fFF.get(); 
    kMaxOutput = ShooterConstants.flywheelMaxOutput;
    kMinOutput = ShooterConstants.flywheelMinOutput;
    maxRPM = ShooterConstants.flywheelMaxRPM; 

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
    mm_sVelocity.setDefault(ShooterConstants.shooterVelocity);
    mm_sAcceleration.setDefault(ShooterConstants.shooterAcceleration);
    mm_sJerk.setDefault(ShooterConstants.shooterJerk);

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = mm_sVelocity.get();
    mm.MotionMagicAcceleration = mm_sAcceleration.get();
    mm.MotionMagicJerk = mm_sJerk.get();

    // PID Coefficients for Shooter Angle
    sP.setDefault(ShooterConstants.shooterP);
    sI.setDefault(ShooterConstants.shooterI);
    sD.setDefault(ShooterConstants.shooterD);
    sV.setDefault(ShooterConstants.shooterV);
    sS.setDefault(ShooterConstants.shooterS);

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = sP.get();
    slot0.kI = sI.get();
    slot0.kD = sD.get();
    slot0.kV = sV.get();
    slot0.kS = sS.get();

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

  public void flywheelsOnSub() {
    m_pidController.setReference(subRPM.get()*Math.PI, CANSparkMax.ControlType.kVelocity);
  }
  public void flywheelsOnPod() {
    m_pidController.setReference(podRPM.get()*Math.PI, CANSparkMax.ControlType.kVelocity);
  }

  public void intakeHP() {
    flywheelLeft.set(intakeHPspeed.get());
  }

  public void disableFlywheels() {
    flywheelLeft.set(0);
  }

  public void lowerToIntake() {
    shooter.setControl(m_mmReq.withPosition(intakingAngle.get()).withSlot(0));
  }

  public void lowerToShootSub() {
    shooter.setControl(m_mmReq.withPosition(subAngle.get()).withSlot(0));
  }

  public void lowerToShootPod() {
    shooter.setControl(m_mmReq.withPosition(podAngle.get()).withSlot(0));
  }

  public void interpolatedFlywheelVelocity(State state) {
    m_pidController.setReference(state.speed*Math.PI, CANSparkMax.ControlType.kVelocity);
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
    return (shooter_sensor.getRange() < shooterSensorRange.get());
  }

  public boolean invShooterSensorOut() {
    return (shooter_sensor.getRange() > shooterSensorRange.get());
  }

  public boolean isSubwooferVelocitySet() {
    return((m_encoder.getVelocity() >= (subRPM.get())-fThreshold.get()) && (m_encoder.getVelocity() <= ((subRPM.get())+fThreshold.get())));
  }

  public boolean isPodiumVelocitySet() {
    return((m_encoder.getVelocity() >= (podRPM.get())-fThreshold.get()) && (m_encoder.getVelocity() <= ((podRPM.get())+fThreshold.get())));
  }

  public boolean isInterpolatedVelocitySet(State state) {
    return((m_encoder.getVelocity() >= (state.speed)-fThreshold.get()) && (m_encoder.getVelocity() <= ((state.speed)+fThreshold.get())));
  }
  public double a(State state) {
    return (state.speed)-fThreshold.get();
  }

  public boolean isAngleSet() {
    return (m_mmReq.Position <= (shooter.getPosition().getValueAsDouble()+sThreshold.get()) && m_mmReq.Position >= (shooter.getPosition().getValueAsDouble()-sThreshold.get()));
  }

  public void periodic() {
    SmartDashboard.putNumber("Shooter Angle Position: ", shooter.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Angle Velocity: ", shooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Angle Power:", shooter.get());
    SmartDashboard.putNumber("Shooter Angle Voltage:", shooter.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putNumber("Flywheels Velocity: ", m_encoder.getVelocity());
    SmartDashboard.putNumber("Flywheels Power:", flywheelLeft.get());

    SmartDashboard.putBoolean("Shooter Angle Setpoint Reached: ", isAngleSet());
  }

}
