package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.shooter.Shooter.State;

/**
 * Place to hold robot-wide numerical or boolean constants. This class should not be used for any
 * other purpose. All constants should be declared globally (i.e. public static). Do not put
 * anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum TeleOpMode {
    // Sensor-Enabled Shooter Scoring Mode
    AUTO_SPEAKER,

    // Sensor-Enabled Amp Scoring Mode
    AUTO_AMP,

    // Manual Shooter Scoring Mode
    MANUAL_SPEAKER,

    // Manual Amp Scoring Mode
    MANUAL_AMP,
  }
  public static final List<Entry<Measure<Distance>,State>> SHOOTER_MAP = Arrays.asList(
      Map.entry(Units.Meters.of(0.0), new State(0.2, 3.3)),
      Map.entry(Units.Meters.of(1.0), new State(0.4,3.6)),
      Map.entry(Units.Meters.of(2.0), new State(0.6,3.9)),
      Map.entry(Units.Meters.of(3.0), new State(0.6,4.3)),
      Map.entry(Units.Meters.of(4.0), new State(0.6,4.5)),
      Map.entry(Units.Meters.of(5.0), new State(0.6,4.8)));

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
}

