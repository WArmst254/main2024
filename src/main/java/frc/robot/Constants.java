package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;                           
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.PolynomialRegression;

/**
 * Place to hold robot-wide numerical or boolean constants. This class should
 * not be used for any
 * other purpose. All constants should be declared globally (i.e. public
 * static). Do not put
 * anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final boolean tuningMode = true;

  public static final double kConfigTimeoutSeconds = 0.1;

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double AprilTagHeights[] = {
      53.38,
      53.38,
      57.13,
      57.13,
      53.38,
      53.38,
      57.13,
      57.13,
      53.38,
      53.38,
      52,
      52,
      52,
      52,
      52,
      52
  };

   // https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
  public static class AprilTags {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static int BLUE_SOURCE_LEFT = 1;
    public static int BLUE_SOURCE_RIGHT = 2;
    public static int RED_SPEAKER_BOTTOM = 3;
    public static int RED_SPEAKER_TOP = 4;
    public static int RED_AMP = 5;
    public static int BLUE_AMP = 6;
    public static int BLUE_SPEAKER_TOP = 7;
    public static int BLUE_SPEAKER_BUTTON = 8;
    public static int RED_SOURCE_LEFT = 9;
    public static int RED_SOURCE_RIGHT = 10;
    public static int RED_STAGE_BOTTOM = 11;
    public static int RED_STAGE_TOP = 12;
    public static int RED_STAGE_SIDE = 13;
    public static int BLUE_STAGE_SIDE = 14;
    public static int BLUE_STAGE_TOP = 15;
    public static int BLUE_STAGE_BOTTOM = 16;
  }

  public static class Setpoints {

    public static final double PivotStowAngle = 0;
    public static final double ElevatorStowHeight = 0;

    public static final double indexingTargetVolts = 3; 
    public static final double indexingTargetVoltsSlow = 1;
    public static final double indexerScoringVoltage = 5;

    public static final double intakingTargetVoltage = 6;
    public static final double outtakingTargetVoltage = -6;

    public static final double pivotMinClamp = 0;
    public static final double pivotMaxClamp = 60;

    public static final double shooterMinClamp = 0;
    public static final double shooterMaxClamp = 6000;

  }

  public static final class AmpConstants {

    public static final int ampTalonID = 19; 
    public static final String ampTalonCANBus = "rio";

    public static final int ampSensorID = 27; 
    public static final RangingMode ampSensorRange = RangingMode.Short;
    public static final double ampSampleTime = 0;

    public static final double isNotePresentTOF = 250; // Milimeters
    public static final double isNoteCenteredTOF = 70; // Milimeters
    public static final double isNoteCenteredTOFTolerance = 5; // Milimeters

    private static final double ampMaxDutyCycle = 0.5;

    public static final TalonFXConfiguration kAmpConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive));

    public static final DutyCycleOut ampDutyCycle = new DutyCycleOut(0, true, false, false, false);
    public static final TorqueCurrentFOC ampTorqueControl = new TorqueCurrentFOC(0, ampMaxDutyCycle, 0, false, false, false);
  }

  public static final class ElevatorConstants {

    public static final int elevatorLeaderTalonID = 22; 
    public static final int elevatorFollowerTalonID = 23;
    public static final String elevatorTalonCANBus = "rio";

    public static final double elevatorGearRatio = 25; // Sensor to Mechanism Ratio
    public static final double elevatorPinionRadius = Units.inchesToMeters(1); // Meters

    public static final double maxElevatorHeight = 0.46; // Meters

    public static final TalonFXConfiguration kElevatorConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(false))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive))
      .withSlot0(new Slot0Configs()
        .withKV(0)
        .withKA(0)
        .withKP(15)
        .withKI(0)
        .withKD(0)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(0.35))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(elevatorGearRatio))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(6)
        .withMotionMagicAcceleration(18)
        .withMotionMagicJerk(0))
      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(false)
        .withForwardSoftLimitThreshold(elevatorMetersToRotations(maxElevatorHeight))
        .withReverseSoftLimitThreshold(0));

    public static final MotionMagicTorqueCurrentFOC elevatorPositionControl = new MotionMagicTorqueCurrentFOC(0, 0, 0, false, false, false);

    public static final Follower followerControl = new Follower(elevatorLeaderTalonID, true);

    public static final double heightErrorTolerance = 0.005; // Meters

    public static final double kElevatorFastUpdateFrequency = 50; // Hertz
    public static final double kElevatorMidUpdateFrequency = 40; // Hertz

    public static double elevatorMetersToRotations(double meters) {

            return meters / (2 * Math.PI * elevatorPinionRadius);
    }

    public static double elevatorRotationsToMeters(double rotations) {

            return rotations * (2 * Math.PI * elevatorPinionRadius);
    }
  }

  public static final class FeederConstants {

    public static final int feederTalonID = 19; 
    public static final String feederTalonCANBus = "rio";

    public static final int feederSensorID = 27; 
    public static final RangingMode feederSensorRange = RangingMode.Short;
    public static final double feederSfeederleTime = 0;

    public static final double isNotePresentTOF = 250; // Milimeters
    public static final double isNoteCenteredTOF = 70; // Milimeters
    public static final double isNoteCenteredTOFTolerance = 5; // Milimeters

    private static final double feederMaxDutyCycle = 0.5;

    public static final TalonFXConfiguration kFeederConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive));

    public static final DutyCycleOut feederDutyCycle = new DutyCycleOut(0, true, false, false, false);

    public static final TorqueCurrentFOC feederTorqueControl = new TorqueCurrentFOC(0, feederMaxDutyCycle, 0, false, false, false);
  }

  public static final class IntakeConstants {

    public static final int intakeTalonID = 18;
    public static final String intakeTalonCANBus = "rio";

    public static final int intakeSensorID = 26;
    public static final RangingMode intakeSensorRange = RangingMode.Short;
    public static final double intakeSampleTime = 24;

    public static final double isNotePresentTOF = 250; // Milimeters

    private static final double intakeMaxDutyCycle = 0.5;

    public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(false))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive));

    public static final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0, true, false, false, false);

    public static final TorqueCurrentFOC intakeTorqueControl = new TorqueCurrentFOC(0, intakeMaxDutyCycle, 0, false, false, false);

  }

  public static final class PivotConstants {

    public static final int pivotTalonID =  16; 
    public static final String pivotTalonCANBus = "rio";
    public static final int pivotEncoderID = 20;
    public static final String pivotEncoderCANBus = "rio";

    public static final double pivotGearRatio = 15 * 2; // Sensor to Mechanism Ratio
    public static final double timeBeforeEncoderReset = 1.5; // Seconds before the motor is initialized to the through bore;

    public static final int pivotEncoderPort = 0; 
    public static final Rotation2d absoluteEncoderOffset = Rotation2d.fromRotations(0.5);
    /****
    * Pivot is counter-clockwise (from the left side of the robot with intake
    * forward) with 0 being horizontal towards the intake
    ****/
    public static final Rotation2d pivotMinAngle = Rotation2d.fromDegrees(-25);
    public static final Rotation2d pivotMaxAngle = Rotation2d.fromDegrees(180);

    public static final TalonFXConfiguration kPivotConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive))
      .withSlot0(new Slot0Configs()
        .withKV(0)
        .withKA(0)
        .withKP(55) // 40  || 55
        .withKI(0)
        .withKD(3) // 0.5 || 5
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(-0.4) // -0.4 // Negative b/c of pivot direction & how CTRE uses it
        .withKS(0)) // 0.5
      .withFeedback(new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withFeedbackRemoteSensorID(pivotEncoderID)
        .withSensorToMechanismRatio(1)
        .withRotorToSensorRatio(pivotGearRatio))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(10) // Default 10
        .withMotionMagicAcceleration(15) // Default 15
        .withMotionMagicJerk(0))
      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(pivotMaxAngle.getRotations())
        .withReverseSoftLimitThreshold(pivotMinAngle.getRotations()));

    public static final CANcoderConfiguration kPivotEncoderConfiguration = new CANcoderConfiguration()
    .withMagnetSensor(new MagnetSensorConfigs()
      .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
      .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    public static final MotionMagicTorqueCurrentFOC pivotPositionControl = new MotionMagicTorqueCurrentFOC(0, 0, 0, false, false, false);

    public static final double kPivotPositionUpdateFrequency = 50; // Hertz
    public static final double kPivotErrorUpdateFrequency = 50; // Hertz

    public static final Rotation2d angleErrorTolerance = Rotation2d.fromDegrees(0.4); // Degrees
  }

  public static final class ShooterConstants {

    public static final int shooterTalonLeaderID = 16; 
    public static final int shooterTalonFollowerID = 17;
    public static final String shooterTalonCANBus = "rio";

    public static final double shooterGearRatio = 0.5; // Sensor to Mechanism Ratio

    public static final double shooterVelocityTolerance = 200; // RPM

    public static final TalonFXConfiguration kShooterConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(120)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(false))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive))
      .withSlot0(new Slot0Configs()
        .withKV(0.071) // 0.075
        .withKP(0.5) // 0.125
        .withKI(0)
        .withKD(0))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(shooterGearRatio));

    public static final VelocityTorqueCurrentFOC shooterControl = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

    public static final Follower followerControl = new Follower(shooterTalonLeaderID, true);

    public static final double kShooterVelocityUpdateFrequency = 10; // Hertz
  }

  public static double[][] kRPMValues = {
    {6.7, 1084},
    {5.5, 990},
    {4.5, 900},
    {3.5, 813},
    {2.5, 770},
    {2, 730},
    {1.5, 740},
};

public static double[][] kPivotValues = {
    {6.7, 25},
    {5.5, 21.66},
    {4.5, 19.2},
    {3.5, 12},
    {2.5, 7},
    {2, 4.5},
    {1.5, 0}
};

public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPivotMap = new InterpolatingTreeMap<>();
public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap = new InterpolatingTreeMap<>();

public static PolynomialRegression kPivotRegression;
public static PolynomialRegression kRPMRegression;

static {
    for (double[] pair : kRPMValues) {
        kRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }

    for (double[] pair : kPivotValues) {
        kPivotMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }

    kPivotRegression = new PolynomialRegression(kPivotValues, 1);
    kRPMRegression = new PolynomialRegression(kRPMValues, 1);
}

public static final class DriveConstants {
  public static final double shootingDriveScalar = 0.1;
}

  public static final class IDs {

    // Tuner X
    public static final int pigeon = 0;
    public static final int swerveDriveTalon0 = 1;
    public static final int swerveTurnTalon0 = 2;
    public static final int swerveDriveTalon1 = 3;
    public static final int swerveTurnTalon1 = 4;
    public static final int swerveDriveTalon2 = 5;
    public static final int swerveTurnTalon2 = 6;
    public static final int swerveDriveTalon3 = 7;
    public static final int swerveTurnTalon3 = 8;
    public static final int swerveCANcoder0 = 9;
    public static final int swerveCANcoder1 = 10;
    public static final int swerveCANcoder2 = 11;
    public static final int swerveCANcoder3 = 12;
  }

}
