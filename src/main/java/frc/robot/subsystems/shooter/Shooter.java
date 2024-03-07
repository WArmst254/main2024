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

  private TunableNumber tRPM = new TunableNumber("TEST RPM");
  private TunableNumber tAngle = new TunableNumber("TEST ANGLE");

  private TunableNumber mm_sVelocity = new TunableNumber("Shooter Angle/Velocity");
  private TunableNumber mm_sAcceleration = new TunableNumber("Shooter Angle/Acceleration");
  private TunableNumber mm_sJerk = new TunableNumber("Shooter Angle/Jerk");
  private TunableNumber sP = new TunableNumber("Shooter Angle PID/P");
  private TunableNumber sI = new TunableNumber("Shooter Angle PID/I");
  private TunableNumber sD = new TunableNumber("Shooter Angle PID/D");
  private TunableNumber sV = new TunableNumber("Shooter Angle PID/V");
  private TunableNumber sS = new TunableNumber("Shooter FlyWheel PID/S");

  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private PolynomialSplineFunction m_shooterCurve;
  private PolynomialSplineFunction m_flywheelCurve;
  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;

  public Shooter() {

    tRPM.setDefault(2500);
    tAngle.setDefault(0);
    
    /*Flywheel Settings and Gains */
    flywheelLeft.restoreFactoryDefaults();
    flywheelRight.restoreFactoryDefaults();
    flywheelRight.follow(flywheelLeft);
    flywheelRight.burnFlash();
  
    fP.setDefault(6e-5);
    fI.setDefault(0);
    fD.setDefault(0);
    fIz.setDefault(0);
    fFF.setDefault(0.000015);
    
    // PID coefficients for Flywheels
    kP = fP.get();
    kI = fI.get();
    kD = fD.get();
    kIz = fIz.get(); 
    kFF = fFF.get(); 
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 6000; 

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
    mm_sVelocity.setDefault(30);
    mm_sAcceleration.setDefault(30);
    mm_sJerk.setDefault(100);

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = mm_sVelocity.get();
    mm.MotionMagicAcceleration = mm_sAcceleration.get();
    mm.MotionMagicJerk = mm_sJerk.get();

    // PID Coefficients for Shooter Angle
    sP.setDefault(30);
    sI.setDefault(20);
    sD.setDefault(0.12);
    sV.setDefault(0.12);
    sS.setDefault(0.25);

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

  public void flywheelsOn() {
    m_pidController.setReference(tRPM.get()*Math.PI, CANSparkMax.ControlType.kVelocity);
  }

  public void intakeHP() {
    flywheelLeft.set(-0.2);
  }

  public void disableFlywheels() {
    flywheelLeft.set(0);
  }

  public void lowerToIntake() {
    shooter.setControl(m_mmReq.withPosition(0.2).withSlot(0));
  }

  public void lowerToShoot() {
    shooter.setControl(m_mmReq.withPosition(tAngle.get()).withSlot(0));
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
    return (shooter_sensor.getRange() < 335);
  }

  public boolean invShooterSensorOut() {
    return !(shooter_sensor.getRange() < 335);
  }

  public boolean isVelocitySet() {
    return((m_encoder.getVelocity() >= (tRPM.get())-100) && (m_encoder.getVelocity() <= ((tRPM.get())+100)));
  }

  public boolean isInterpolatedVelocitySet(State state) {
    return((m_encoder.getVelocity() >= (state.speed)-100) && (m_encoder.getVelocity() <= ((state.speed)+100)));
  }

  public boolean isAngleSet() {
    return (m_mmReq.Position <= (shooter.getPosition().getValueAsDouble()+0.05) && m_mmReq.Position >= (shooter.getPosition().getValueAsDouble()-0.05));
  }

  public boolean isShooterSet() {
    return isVelocitySet() && isAngleSet();
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
