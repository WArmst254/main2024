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
    public static final int frontFeed = 13;
    public static final int backFeed = 14;
    public static final int elevator = 15;
    public static final int shooter = 16;
    public static final int intake = 17;
    public static final int amp = 18;
    public static final int led = 20;

    // REV Hardware Client
    public static final int flywheelLeft = 19;
    public static final int flywheelRight = 20;
    
    //PWF Config Page 10.49.44.2:5812
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


  public static final List<Entry<Measure<Distance>,State>> SHOOTER_MAP = Arrays.asList(
    Map.entry(Units.Meters.of(0), new State(2500, 0)),
    Map.entry(Units.Meters.of(1.215), new State(3300, 0)),
    Map.entry(Units.Meters.of(1.697), new State(3600,0.095)),
    Map.entry(Units.Meters.of(2.000), new State(3700,0.133)),
    Map.entry(Units.Meters.of(2.709), new State(3700,0.206)),
    Map.entry(Units.Meters.of(3.596), new State(3900,0.242)),
    Map.entry(Units.Meters.of(3.620), new State(3910,0.248)));

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

