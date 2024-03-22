package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.shooter.Shooter.State;

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
    public static final int pivotCANcoder = 61;
    public static final int frontFeed = 13;
    public static final int backFeed = 14;
    public static final int leadElevator = 30;
    public static final int followerElevator = 31;
    public static final int shooter = 16;
    public static final int intake = 17;
    public static final int amp = 18;
    public static final int led = 20;

    // REV Hardware Client
    public static final int flywheelLeft = 19;
    public static final int flywheelRight = 20;

    // PWF Config Page 10.49.44.2:5812
    public static final int intakesensor = 1;
    public static final int shootersensor = 2;
    public static final int ampsensor = 3;

  }


  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final List<Entry<Measure<Distance>, State>> SHOOTER_MAP = Arrays.asList(
      Map.entry(Units.Meters.of(0), new State(2500, 0)),
      Map.entry(Units.Meters.of(1.225), new State(3100, 0)),
      Map.entry(Units.Meters.of(1.707), new State(3400, 0.095)),
      Map.entry(Units.Meters.of(2.010), new State(3500, 0.133)),
      Map.entry(Units.Meters.of(2.540), new State(3500, 0.19)),
      Map.entry(Units.Meters.of(2.719), new State(4500, 0.206)),
      Map.entry(Units.Meters.of(3.606), new State(3700, 0.242)),
      Map.entry(Units.Meters.of(3.630), new State(3700, 0.248)));

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

  public class XboxConstants {
    // Joystick Axises
    public static final int L_JOYSTICK_HORIZONTAL = 0;
    public static final int L_JOYSTICK_VERTICAL = 1;
    public static final int LT = 2;
    public static final int RT = 3;
    public static final int R_JOYSTICK_HORIZONTAL = 4;
    public static final int R_JOYSTICK_VERTICAL = 5;

    // Controller Buttons
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LB_BUTTON = 5;
    public static final int RB_BUTTON = 6;
    public static final int SELECT_BUTTON = 7;
    public static final int START_BUTTON = 8;

    // These buttons are when you push down the left and right circle pad
    public static final int L_JOYSTICK_BUTTON = 9;
    public static final int R_JOYSTICK_BUTTON = 10;

    // D Pad Buttons
    public static final int DPAD_UP = 0;
    public static final int DPAD_UP_RIGHT = 45;
    public static final int DPAD_RIGHT = 90;
    public static final int DPAD_DOWN_RIGHT = 135;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_DOWN_LEFT = 225;
    public static final int DPAD_LEFT = 270;
    public static final int DPAD_UP_LEFT = 315;

    // Controller Zeroes
    public static final double ZERO = 0.15;
  }

  /**
   * Gotten from here
   * https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
   */
  public static class AprilTags {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo
        .loadAprilTagLayoutField();

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

  public static Pose2d mirrorPose(Pose2d bluePose) {
    return new Pose2d(
        Constants.AprilTags.aprilTagFieldLayout.getFieldLength() - bluePose.getX(),
        bluePose.getY(),
        Rotation2d.fromRadians(Math.PI - bluePose.getRotation().getRadians()));
  }

  public static class PoseConfig {
    // Increase these numbers to trust your model's state estimates less.
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;

    // Increase these numbers to trust global measurements from vision less.
    public static final double kVisionStdDevX = 5;
    public static final double kVisionStdDevY = 5;
    public static final double kVisionStdDevTheta = 500;
  }
}
