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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.FieldUtil;
import frc.robot.util.PoseTracker;
import frc.robot.util.TunableNumber;

public class Shooter extends SubsystemBase {

  //Hardware
  private final CANSparkFlex flywheelLeft;
  private final CANSparkFlex flywheelRight;
  private final TalonFX shooter;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  CANcoder absoluteEncoder = new CANcoder(60);
  public final TimeOfFlight shooter_sensor;

  //Control
  private final SparkPIDController leftController;
  private final SparkPIDController rightController;
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.335, 0.00175, 0.0);
  private final MotionMagicVoltage m_mmReq;

  // Status Signals
  private final StatusSignal<Double> internalPositionRotations;
  private final StatusSignal<Double> encoderAbsolutePositionRotations;
  private final StatusSignal<Double> encoderRelativePositionRotations;
  private final StatusSignal<Double> velocityRps;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  //Tunable Numbers -- f_ is Flywheel, s_ is Shooter (Pivot)
  private TunableNumber fP = new TunableNumber("FlyWheel/PID/P");
  private TunableNumber fI = new TunableNumber("FlyWheel/PID/I");
  private TunableNumber fD = new TunableNumber("FlyWheel/PID/D");
  private TunableNumber fIz = new TunableNumber("FlyWheel/PID/IZ");
  private TunableNumber fFF = new TunableNumber("FlyWheel/PID/FF");
  private TunableNumber fTolerance = new TunableNumber("Flywheel/PID/Tolerance(RPM)");

  private TunableNumber sVelocity = new TunableNumber("Angle/PID/Velocity");
  private TunableNumber sAcceleration = new TunableNumber("Angle/PID/Acceleration");
  private TunableNumber sJerk = new TunableNumber("Angle/PID/Jerk");
  private TunableNumber sP = new TunableNumber("Angle/PID/P");
  private TunableNumber sI = new TunableNumber("Angle/PID/I");
  private TunableNumber sD = new TunableNumber("Angle/PID/D");
  private TunableNumber sV = new TunableNumber("Angle/PID/V");
  private TunableNumber sS = new TunableNumber("Angle/PID/S");
  private TunableNumber sTolerance = new TunableNumber("Angle/PID/Tolerance");
  
  private TunableNumber subRPM = new TunableNumber("Flywheel/Setpoints/Subwoofer(RPM)");
  private TunableNumber podRPM = new TunableNumber("Flywheel/Setpoints/Podium (RPM)");
  private TunableNumber intakeHPspeed = new TunableNumber("Flywheel/Setpoints/HPIntake(Speed)");
  private TunableNumber subAngle = new TunableNumber("Angle/Setpoints/Subwoofer");

  private TunableNumber podAngle = new TunableNumber("Angle/Setpoints/Podium");
  private TunableNumber intakingAngle = new TunableNumber("Angle/Setpoints/Intaking");
  private TunableNumber shooterSensorThreshold = new TunableNumber("Sensors/ShooterSensorRange");

  //Interpolation Curves and Limits
  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private PolynomialSplineFunction m_shooterCurve;
  private PolynomialSplineFunction m_flywheelCurve;
  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;
  
  public Shooter() {

    /*Sensor Shtuff */
    shooter_sensor = new TimeOfFlight(Constants.IDs.shootersensor);
    shooter_sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.001);
    shooter_sensor.setRangeOfInterest(6, 6, 14, 14);
    shooterSensorThreshold.setDefault(ShooterConstants.sensorThreshold);

    /*Flywheel Shtuff */
    // Config Hardware
    flywheelRight = new CANSparkFlex(Constants.IDs.flywheelRight, CANSparkFlex.MotorType.kBrushless);
    flywheelLeft = new CANSparkFlex(Constants.IDs.flywheelLeft, CANSparkFlex.MotorType.kBrushless);
    rightEncoder= flywheelRight.getEncoder();
    leftEncoder = flywheelLeft.getEncoder();
    
    // Defaults
    flywheelLeft.restoreFactoryDefaults();
    flywheelRight.restoreFactoryDefaults();

    // Limits
    flywheelLeft.setSmartCurrentLimit(60);
    flywheelRight.setSmartCurrentLimit(60);
    flywheelLeft.enableVoltageCompensation(12.0);
    flywheelRight.enableVoltageCompensation(12.0);

    // Reset encoders
    leftEncoder.setMeasurementPeriod(10);
    rightEncoder.setMeasurementPeriod(10);
    leftEncoder.setAverageDepth(2);
    rightEncoder.setAverageDepth(2);

    // Get controllers
    rightController = flywheelRight.getPIDController();
    leftController = flywheelLeft.getPIDController();
    
    // Disable brake mode
    flywheelLeft.setIdleMode(CANSparkBase.IdleMode.kCoast);
    flywheelRight.setIdleMode(CANSparkBase.IdleMode.kCoast);

    flywheelRight.burnFlash();
    flywheelLeft.burnFlash();

    fP.setDefault(ShooterConstants.flywheelP);
    fI.setDefault(ShooterConstants.flywheelI);
    fD.setDefault(ShooterConstants.flywheelD);
    fIz.setDefault(ShooterConstants.flywheelIZ);
    fFF.setDefault(ShooterConstants.flywheelFF);
    fTolerance.setDefault(ShooterConstants.flywheelTolerance);
    subRPM.setDefault(ShooterConstants.subwooferRPM);
    podRPM.setDefault(ShooterConstants.podiumRPM);
    
    kMaxOutput = ShooterConstants.flywheelMaxOutput;
    kMinOutput = ShooterConstants.flywheelMinOutput;
    maxRPM = ShooterConstants.flywheelMaxRPM; 

    rightController.setP(fP.get());
    rightController.setI(fI.get());
    rightController.setD(fD.get());
    rightController.setIZone(fIz.get());
    rightController.setOutputRange(kMinOutput, kMaxOutput);
    leftController.setP(fP.get());
    leftController.setI(fI.get());
    leftController.setD(fD.get());
    leftController.setIZone(fIz.get());
    leftController.setOutputRange(kMinOutput, kMaxOutput);

    /*Pivot Shtuff */
    shooter = new TalonFX(Constants.IDs.shooter);
    m_mmReq = new MotionMagicVoltage(0);

      // Shooter Encoder Configs
    CANcoderConfiguration shooterEncoderConfig = new CANcoderConfiguration();
    shooterEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    shooterEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    shooterEncoderConfig.MagnetSensor.MagnetOffset = 0.191;
    absoluteEncoder.getConfigurator().apply(shooterEncoderConfig, 1.0);
    shooter.setNeutralMode(NeutralModeValue.Brake);

    //Pivot Motor Configs
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    /* Motion Profiling Constants For Shooter Angle*/
    sVelocity.setDefault(ShooterConstants.shooterVelocity);
    sAcceleration.setDefault(ShooterConstants.shooterAcceleration);
    sJerk.setDefault(ShooterConstants.shooterJerk);

    MotionMagicConfigs mm = config.MotionMagic;
    mm.MotionMagicCruiseVelocity = sVelocity.get();
    mm.MotionMagicAcceleration = sAcceleration.get();
    mm.MotionMagicJerk = sJerk.get();

    // PID Coefficients for Shooter Angle
    sP.setDefault(ShooterConstants.shooterP);
    sI.setDefault(ShooterConstants.shooterI);
    sD.setDefault(ShooterConstants.shooterD);
    sV.setDefault(ShooterConstants.shooterV);
    sS.setDefault(ShooterConstants.shooterS);

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = sP.get();
    slot0.kI = sI.get();
    slot0.kD = sD.get();
    slot0.kV = sV.get();
    slot0.kS = sS.get();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    shooter.getConfigurator().apply(config, 1.0);

    internalPositionRotations = shooter.getPosition();
    encoderAbsolutePositionRotations = absoluteEncoder.getAbsolutePosition();
    encoderRelativePositionRotations = absoluteEncoder.getPosition();
    velocityRps = shooter.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
      100,
      internalPositionRotations,
      velocityRps
    );

    BaseStatusSignal.setUpdateFrequencyForAll(
      1000, 
      encoderAbsolutePositionRotations,
      encoderRelativePositionRotations
    );

    subAngle.setDefault(ShooterConstants.subwooferAngle);
    podAngle.setDefault(ShooterConstants.podiumAngle);
    intakeHPspeed.setDefault(ShooterConstants.humanPlayerIntakeSpeed);
    intakingAngle.setDefault(ShooterConstants.groundIntakeAngle);
    sTolerance.setDefault(ShooterConstants.angleTolerance);

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

  public State getAutomaticState() {
    var targetDistance = getTargetDistance();
    SmartDashboard.putNumber("POSE DISTANCE", targetDistance.magnitude());                                    
    var speed = m_flywheelCurve.value(targetDistance.magnitude());
    var angle = m_shooterCurve.value(targetDistance.magnitude());

    return new State(speed, angle);
  }

  private Measure<Distance> getTargetDistance() {
     Translation2d target = FieldUtil.getAllianceSpeakerPosition();
    return Units.Meters.of(
      MathUtil.clamp(
        PoseTracker.field.getRobotPose().getTranslation().getDistance(target),
        MIN_SHOOTING_DISTANCE.in(Units.Meters),
        MAX_SHOOTING_DISTANCE.in(Units.Meters)
      )
    );
  }

  public void flywheelsOnSub() {
    leftController.setReference(
      subRPM.get() * 3,
      CANSparkBase.ControlType.kVelocity,
      0,
      feedforward.calculate(subRPM.get()),
      SparkPIDController.ArbFFUnits.kVoltage);
  rightController.setReference(
      subRPM.get() * 3,
      CANSparkBase.ControlType.kVelocity,
      0,
      feedforward.calculate(subRPM.get()),
      SparkPIDController.ArbFFUnits.kVoltage);
  }

  public void flywheelsOnPod() {
    leftController.setReference(
      podRPM.get() * 3,
      CANSparkBase.ControlType.kVelocity,
      0,
      feedforward.calculate(podRPM.get()),
      SparkPIDController.ArbFFUnits.kVoltage);
  rightController.setReference(
      podRPM.get() * 3,
      CANSparkBase.ControlType.kVelocity,
      0,
      feedforward.calculate(podRPM.get()),
      SparkPIDController.ArbFFUnits.kVoltage);
  }

  public void intakeHP() {
    flywheelLeft.set(intakeHPspeed.get());
    flywheelRight.set(intakeHPspeed.get());
  }

  public void disableFlywheels() {
    flywheelLeft.set(0);
    flywheelRight.set(0);
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
    leftController.setReference(
      state.speed * 3,
      CANSparkBase.ControlType.kVelocity,
      0,
      feedforward.calculate(state.speed),
      SparkPIDController.ArbFFUnits.kVoltage);
  rightController.setReference(
      state.speed * 3,
      CANSparkBase.ControlType.kVelocity,
      0,
      feedforward.calculate(state.speed),
      SparkPIDController.ArbFFUnits.kVoltage);
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
    return (shooter_sensor.getRange() < shooterSensorThreshold.get());
  }

  public boolean isSubwooferVelocitySet() {
    return((leftEncoder.getVelocity() >= (subRPM.get())-fTolerance.get()) && (leftEncoder.getVelocity() <= ((subRPM.get())+fTolerance.get()))
    && ((rightEncoder.getVelocity() >= (subRPM.get())-fTolerance.get()) && (rightEncoder.getVelocity() <= ((subRPM.get())+fTolerance.get()))));
  }

  public boolean isPodiumVelocitySet() {
    return((leftEncoder.getVelocity() >= (podRPM.get())-fTolerance.get()) && (leftEncoder.getVelocity() <= ((podRPM.get())+fTolerance.get()))
    && ((rightEncoder.getVelocity() >= (podRPM.get())-fTolerance.get()) && (rightEncoder.getVelocity() <= ((podRPM.get())+fTolerance.get()))));
  }

  public boolean isInterpolatedVelocitySet(State state) {
    return((leftEncoder.getVelocity() >= (state.speed)-fTolerance.get()) && (leftEncoder.getVelocity() <= ((state.speed)+fTolerance.get()))
    && ((rightEncoder.getVelocity() >= (state.speed)-fTolerance.get()) && (rightEncoder.getVelocity() <= ((state.speed)+fTolerance.get()))));
  }

  public boolean isAngleSet() {
    return (m_mmReq.Position <= (absoluteEncoder.getAbsolutePosition().getValueAsDouble()+sTolerance.get()) && m_mmReq.Position >= (absoluteEncoder.getAbsolutePosition().getValueAsDouble()-sTolerance.get()));
  }

  public void periodic() {
    SmartDashboard.putNumber("Angle/Reported Internal Position", internalPositionRotations.getValueAsDouble());
    SmartDashboard.putNumber("Angle/Reported Absolute Position", encoderAbsolutePositionRotations.getValueAsDouble());
    SmartDashboard.putNumber("Angle/Recorded Relative Position", encoderRelativePositionRotations.getValueAsDouble());
    SmartDashboard.putNumber("Angle/Reported Velocity:", shooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Angle/Reported Power:", shooter.get());
    SmartDashboard.putNumber("Angle/ReportedVoltage:", shooter.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putNumber("Flywheel/Reported Left Velocity:", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Flywheel/Reported Left Power:", flywheelLeft.get());
    SmartDashboard.putNumber("Flywheel/Reported Right Velocity:", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Flywheel/Reported Right Power:", flywheelRight.get());

    SmartDashboard.putBoolean("Angle/Boolean Setpoint Reached:", isAngleSet());
  }

}
