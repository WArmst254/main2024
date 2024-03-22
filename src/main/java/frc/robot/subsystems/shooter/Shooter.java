package frc.robot.subsystems.shooter;
//HI WYATT - owen
import java.util.Map.Entry;
import java.util.List;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldUtil;
import frc.robot.util.PoseTracker;
import frc.robot.util.TunableNumber;

public class Shooter extends SubsystemBase {

   // Hardware
   private final TalonFX leftTalon;
   private final TalonFX rightTalon;
  private final TalonFX pivot;
  private final CANcoder absoluteEncoder;
  public final TimeOfFlight shooter_sensor;
 
   // Status Signals
   private final StatusSignal<Double> leftPosition;
   private final StatusSignal<Double> leftVelocity;
   private final StatusSignal<Double> leftAppliedVolts;
   private final StatusSignal<Double> leftSupplyCurrent;
   private final StatusSignal<Double> leftTorqueCurrent;
   private final StatusSignal<Double> leftTempCelsius;
   private final StatusSignal<Double> rightPosition;
   private final StatusSignal<Double> rightVelocity;
   private final StatusSignal<Double> rightAppliedVolts;
   private final StatusSignal<Double> rightSupplyCurrent;
   private final StatusSignal<Double> rightTorqueCurrent;
   private final StatusSignal<Double> rightTempCelsius;

  private final StatusSignal<Double> internalPositionRotations;
  private final StatusSignal<Double> encoderAbsolutePositionRotations;
  private final StatusSignal<Double> encoderRelativePositionRotations;
  private final StatusSignal<Double> velocityRps;
  private final StatusSignal<Double> appliedVoltage;
  private final StatusSignal<Double> supplyCurrent;
  private final StatusSignal<Double> torqueCurrent;
  private final StatusSignal<Double> tempCelsius;

  private Translation2d target;
 
   // Control
  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true);
  private final NeutralOut neutralControl = new NeutralOut();
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0).withEnableFOC(true);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.335, 0.00175, 0.0);

  //Tunable Numbers -- f_ is Flywheels, p_ is Pivot
  private TunableNumber fP = new TunableNumber("Flywheel/PID/P");
  private TunableNumber fI = new TunableNumber("Flywheel/PID/I");
  private TunableNumber fD = new TunableNumber("Flywheel/PID/D");
  private TunableNumber fV = new TunableNumber("Flywheel/PID/V");
  private TunableNumber fS = new TunableNumber("Flywheel/PID/S");
  private TunableNumber fA = new TunableNumber("Flywheel/PID/A");
  private TunableNumber fIntakeHPspeed = new TunableNumber("Flywheel/Setpoints/HPIntake(Speed)");
  private TunableNumber fTolerance = new TunableNumber("Flywheel/PID/Tolerance(RPM)");
  private TunableNumber subRPM = new TunableNumber("Flywheel/Setpoints/Subwoofer(RPM)");
  private TunableNumber podRPM = new TunableNumber("Flywheel/Setpoints/Podium (RPM)");

  private TunableNumber pVelocity = new TunableNumber("Pivot/PID/Velocity");
  private TunableNumber pAcceleration = new TunableNumber("Pivot/PID/Acceleration");
  private TunableNumber pJerk = new TunableNumber("Pivot/PID/Jerk");
  private TunableNumber pP = new TunableNumber("Pivot/PID/P");
  private TunableNumber pI = new TunableNumber("Pivot/PID/I");
  private TunableNumber pD = new TunableNumber("Pivot/PID/D");
  private TunableNumber pV = new TunableNumber("Pivot/PID/V");
  private TunableNumber pS = new TunableNumber("Pivot/PID/S");
  private TunableNumber pTolerance = new TunableNumber("Pivot/PID/Tolerance");
  private TunableNumber pSubAngle = new TunableNumber("Pivot/Setpoints/Subwoofer");
  private TunableNumber pPodAngle = new TunableNumber("Pivot/Setpoints/Podium");
  private TunableNumber pIntakingAngle = new TunableNumber("Pivot/Setpoints/Intaking");

  private TunableNumber shooterSensorThreshold = new TunableNumber("Sensors/ShooterSensorRange");

  //Interpolation Curves and Limits
  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private PolynomialSplineFunction m_shooterCurve;
  private PolynomialSplineFunction m_flywheelCurve;
  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;
  
  public Shooter() {

    //Hardware
    leftTalon = new TalonFX(Constants.IDs.flywheelLeft);
    rightTalon = new TalonFX(Constants.IDs.flywheelRight);
    pivot = new TalonFX(Constants.IDs.shooter);
    absoluteEncoder = new CANcoder(60);
        
    //TOF Sensor Config
    shooter_sensor = new TimeOfFlight(Constants.IDs.shootersensor);
    shooter_sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.001);
    shooter_sensor.setRangeOfInterest(6, 6, 14, 14);
    shooterSensorThreshold.setDefault(ShooterConstants.sensorThreshold);

    // Flywheel General config
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.Feedback.SensorToMechanismRatio = 1;

    fTolerance.setDefault(200);
    subRPM.setDefault(2000);

    fP.setDefault(0.9);
    fI.setDefault(0.0001);
    fD.setDefault(0);
    fV.setDefault(0.16);
    fS.setDefault(0.001);
    fA.setDefault(0);

    // Flywheel Controller config
    controllerConfig.kP = fP.get();
    controllerConfig.kI = fI.get();
    controllerConfig.kD = fD.get();
    controllerConfig.kS = fS.get();
    controllerConfig.kV = fV.get();
    controllerConfig.kA = fA.get();

    // Apply configs
    leftTalon.getConfigurator().apply(flywheelConfig, 1.0);
    rightTalon.getConfigurator().apply(flywheelConfig, 1.0);
    leftTalon.getConfigurator().apply(controllerConfig, 1.0);
    rightTalon.getConfigurator().apply(controllerConfig, 1.0);

    // Set inverts
    leftTalon.setInverted(false);
    rightTalon.setInverted(false);

    // Set signals
    leftPosition = leftTalon.getPosition();
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftSupplyCurrent = leftTalon.getSupplyCurrent();
    leftTorqueCurrent = leftTalon.getTorqueCurrent();
    leftTempCelsius = leftTalon.getDeviceTemp();

    rightPosition = rightTalon.getPosition();
    rightVelocity = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightSupplyCurrent = rightTalon.getSupplyCurrent();
    rightTorqueCurrent = rightTalon.getTorqueCurrent();
    rightTempCelsius = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftSupplyCurrent,
        leftTorqueCurrent,
        leftTempCelsius,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightSupplyCurrent,
        rightTorqueCurrent,
        rightTempCelsius);

    //Pivot Absolute Encoder Configs
    CANcoderConfiguration shooterEncoderConfig = new CANcoderConfiguration();
    shooterEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    shooterEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    shooterEncoderConfig.MagnetSensor.MagnetOffset = 0.028564;
    ;
    absoluteEncoder.getConfigurator().apply(shooterEncoderConfig, 1.0);

    //Pivot Motor Configs
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    /* Motion Profiling Constants For Pivot */
    pVelocity.setDefault(ShooterConstants.shooterVelocity);
    pAcceleration.setDefault(ShooterConstants.shooterAcceleration);
    pJerk.setDefault(ShooterConstants.shooterJerk);

    MotionMagicConfigs mm = config.MotionMagic;
    mm.MotionMagicCruiseVelocity = pVelocity.get();
    mm.MotionMagicAcceleration = pAcceleration.get();
    mm.MotionMagicJerk = pJerk.get();

    // PID Coefficients for Pivot
    pP.setDefault(ShooterConstants.shooterP);
    pI.setDefault(ShooterConstants.shooterI);
    pD.setDefault(ShooterConstants.shooterD);
    pV.setDefault(ShooterConstants.shooterV);
    pS.setDefault(ShooterConstants.shooterS);
//10:58 or 1:7
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = pP.get();
    slot0.kI = pI.get();
    slot0.kD = pD.get();
    slot0.kV = pV.get();
    slot0.kS = pS.get();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.Feedback.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
    //config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    //config.Feedback.RotorToSensorRatio = 12.8;
    config.Feedback.SensorToMechanismRatio = 12.8;
    pivot.getConfigurator().apply(config, 1.0);

    internalPositionRotations = pivot.getPosition();
    encoderAbsolutePositionRotations = absoluteEncoder.getAbsolutePosition();
    encoderRelativePositionRotations = absoluteEncoder.getPosition();
    velocityRps = pivot.getVelocity();
    appliedVoltage = pivot.getMotorVoltage();
    supplyCurrent = pivot.getSupplyCurrent();
    torqueCurrent = pivot.getTorqueCurrent();
    tempCelsius = pivot.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
      100,
      internalPositionRotations,
      velocityRps,
      appliedVoltage,
      supplyCurrent,
      torqueCurrent,
      tempCelsius
    );

    BaseStatusSignal.setUpdateFrequencyForAll(
      1000, 
      encoderAbsolutePositionRotations,
      encoderRelativePositionRotations
    );

    pSubAngle.setDefault(ShooterConstants.subwooferAngle);
    pPodAngle.setDefault(ShooterConstants.podiumAngle);
    fIntakeHPspeed.setDefault(ShooterConstants.humanPlayerIntakeSpeed);
    pIntakingAngle.setDefault(ShooterConstants.groundIntakeAngle);
    pTolerance.setDefault(ShooterConstants.angleTolerance);

    MAX_SHOOTING_DISTANCE = Constants.SHOOTER_MAP.get(Constants.SHOOTER_MAP.size() - 1).getKey();
    initializeShooterCurves(Constants.SHOOTER_MAP);

  }

  //State for Interpolation
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
    SmartDashboard.putNumber("POSE DISTANCE", targetDistance.magnitude());                                    
    var speed = m_flywheelCurve.value(targetDistance.magnitude());
    var angle = m_shooterCurve.value(targetDistance.magnitude());

    return new State(speed, angle);
  }
  private Measure<Distance> getTargetDistance(Vision vision) {
    Translation2d target = FieldUtil.getAllianceSpeakerPosition();
        return Units.Meters.of(
      MathUtil.clamp(
        vision.getDistance(),
        MIN_SHOOTING_DISTANCE.in(Units.Meters),
        MAX_SHOOTING_DISTANCE.in(Units.Meters)
      )
    );
  }


public void flywheelsOnSub() {
  leftTalon.setControl(
    velocityControl.withVelocity(subRPM.get() / 60.0).withFeedForward(ff.calculate(subRPM.get() / 60.0)));
  rightTalon.setControl(
    velocityControl.withVelocity(subRPM.get() / 60.0).withFeedForward(ff.calculate(subRPM.get() / 60.0)));
  }
  public void flywheelsOnPod() {
    leftTalon.setControl(
      velocityControl.withVelocity(podRPM.get() / 60.0));
  rightTalon.setControl(
      velocityControl.withVelocity(podRPM.get() / 60.0));
    }
  public void interpolatedFlywheelVelocity(State state) {
    leftTalon.setControl(
      velocityControl.withVelocity(state.speed / 60.0));
  rightTalon.setControl(
      velocityControl.withVelocity(state.speed / 60.0));
  }
  public void disableFlywheels() {
    leftTalon.setControl(neutralControl);
    rightTalon.setControl(neutralControl);
  }
  public void intakeHP() {
    leftTalon.set(fIntakeHPspeed.get());
    rightTalon.set(fIntakeHPspeed.get());
  }

  public void lowerToIntake() {
    pivot.setControl(m_mmReq.withPosition(pIntakingAngle.get()).withSlot(0));
  }
  public void lowerToShootSub() {
    pivot.setControl(m_mmReq.withPosition(pSubAngle.get()).withSlot(0));
  }
  public void lowerToShootPod() {
    pivot.setControl(m_mmReq.withPosition(pPodAngle.get()).withSlot(0));
  }
  public void interpolatedShooterAngle(State state) {
    pivot.setControl(m_mmReq.withPosition(state.angle).withSlot(0));
  }

  public void stowShooter() {
    pivot.setControl(m_mmReq.withPosition(0).withSlot(0));
  }

  public void zeroShooter() {
    pivot.setPosition(0);
  }

  public double shooterSensor() {
    return shooter_sensor.getRange();
  }
  public boolean shooterSensorOut() {
    return (shooter_sensor.getRange() < shooterSensorThreshold.get());
  }

 public boolean isSubwooferVelocitySet() {
    return((leftVelocity.getValueAsDouble()*60 >= (subRPM.get())-fTolerance.get()) && (leftVelocity.getValueAsDouble()*60 <= ((subRPM.get())+fTolerance.get()))
    && ((rightVelocity.getValueAsDouble()*60 >= (subRPM.get())-fTolerance.get()) && (rightVelocity.getValueAsDouble()*60 <= ((subRPM.get())+fTolerance.get()))));
  }

  public boolean isPodiumVelocitySet() {
    return((leftVelocity.getValueAsDouble())*60 >= (podRPM.get())-fTolerance.get()) && (leftVelocity.getValueAsDouble())*60 <= ((podRPM.get())+fTolerance.get())
    && ((rightVelocity.getValueAsDouble())*60 >= (podRPM.get())-fTolerance.get()) && (rightVelocity.getValueAsDouble())*60 <= ((podRPM.get())+fTolerance.get());
  }

  public boolean isInterpolatedVelocitySet(State state) {
    return((leftVelocity.getValueAsDouble())*60 >= (state.speed)-fTolerance.get()) && (leftVelocity.getValueAsDouble())*60 <= ((state.speed)+fTolerance.get())
    && ((rightVelocity.getValueAsDouble())*60 >= (state.speed)-fTolerance.get()) && (rightVelocity.getValueAsDouble())*60 <= ((state.speed)+fTolerance.get());
  }

  public boolean isAngleSet() {
    return (m_mmReq.Position <= (pivot.getPosition().getValueAsDouble()+pTolerance.get()) && m_mmReq.Position >= (pivot.getPosition().getValueAsDouble()-pTolerance.get()));
  }

  public void periodic() {
   // SmartDashboard.putNumber("Pivot/Reported Internal Position", pivot.getRotorPosition()
    SmartDashboard.putNumber("Pivot/Reported Position", pivot.getPosition().getValueAsDouble());
    //SmartDashboard.putNumber("Pivot/Recorded Relative Position", absoluteEncoder.getPositionSinceBoot().getValueAsDouble());
    SmartDashboard.putNumber("Pivot/Reported Velocity:", pivot.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Pivot/Reported Power:", pivot.get());
    SmartDashboard.putNumber("Pivot/ReportedVoltage:", pivot.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Pivot/Boolean Setpoint Reached:", isAngleSet());

    SmartDashboard.putNumber("Flies L", leftTalon.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("Flies R", rightTalon.getVelocity().getValueAsDouble()*60);
  }

}
