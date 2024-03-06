
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
public final class ShooterConstants {
  SmartDashBoard.getNumber("RPM", 0);
  SmartDashboard.putNumber("RPM", SmartDashBoard.getNumber("RPM", 0));

}

